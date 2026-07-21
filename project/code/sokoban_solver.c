#include "sokoban_solver.h"
#include <string.h>

/*
 * This file implements a small Sokoban solver for a fixed 12x16 board.
 *
 * Core ideas:
 * 1. Store walls / goals / boxes as bitsets to save memory.
 * 2. Search from the start map with A* (normal pushing).
 * 3. Use Manhattan distance as the heuristic.
 * 4. Drop obvious deadlock states early.
 *
 * Reading hint:
 * - "cell" means one grid position in [0, 191]
 * - "boxes/goals" means the compressed bitset of remaining boxes/goals
 */

/* Fixed board size and search memory budget. */
#define SK_CELL_COUNT               (SOKOBAN_MAP_HEIGHT * SOKOBAN_MAP_WIDTH)
#define SK_BITSET_WORDS             (3u)
#define SK_INVALID_NODE             (0xFFFFu)
#define SK_INVALID_CELL             (0xFFu)
#define SK_NODE_CAPACITY            (3072u)
#define SK_HASH_SIZE                (4093u)
#define SK_LABELED_NODE_CAPACITY    (6144u)
#define SK_LABELED_HASH_SIZE        (8191u)
#define SK_SINGLE_STATE_COUNT       (SK_CELL_COUNT * SK_CELL_COUNT)
#define SK_TRACE_UNVISITED          (0xFFu)
#define SK_TRACE_START              (0xFEu)
#define SK_TRACE_PUSH               (0x04u)

#define SK_DIR_UP                   (0u)
#define SK_DIR_DOWN                 (1u)
#define SK_DIR_LEFT                 (2u)
#define SK_DIR_RIGHT                (3u)

#define SK_FLAG_CLOSED              (0x01u)

/* One search node = current box layout + remaining goals + one canonical player position. */
typedef struct
{
    uint64_t boxes[SK_BITSET_WORDS];   /* All box positions. */
    uint64_t goals[SK_BITSET_WORDS];   /* Remaining goal positions. */
    uint16_t parent;                   /* Parent state index for backtracking. */
    uint16_t g;                        /* Cost from start to current state. */
    uint16_t h;                        /* Estimated cost from current state to target. */
    uint16_t hash_next;                /* Next node in the same hash bucket. */
    uint16_t heap_pos;                 /* Position in the binary heap. */
    uint8_t player;                    /* Canonical player cell in reachable area. */
    uint8_t action_from;               /* Player cell before the push that created this node. */
    uint8_t action_dir;                /* Push direction that created this node. */
    uint8_t flags;
} sk_node_t;

/* Parsed puzzle data plus several precomputed helper tables. */
typedef struct
{
    uint64_t wall[SK_BITSET_WORDS];
    uint64_t goal[SK_BITSET_WORDS];
    uint64_t start_boxes[SK_BITSET_WORDS];
    uint64_t dead_square[SK_BITSET_WORDS];
    uint8_t start_player;
    uint8_t box_count;
    uint8_t goal_count;
    uint8_t goal_cells[SOKOBAN_MAX_BOXES];
    uint8_t start_box_cells[SOKOBAN_MAX_BOXES];
} sk_problem_t;

/* Runtime data for one A* direction. */
typedef struct
{
    sk_node_t *nodes;
    uint16_t *heap;
    uint16_t *hash;
    uint16_t node_count;
    uint16_t heap_size;
    uint16_t expanded;
} sk_search_t;

/* Identity-preserving node used by labeled and bomb-aware searches. */
typedef struct
{
    uint8_t box_cells[SOKOBAN_MAX_BOXES];
    uint16_t active_boxes;
    uint16_t remaining_goals;
    uint16_t parent;
    uint16_t g;
    uint16_t h;
    uint16_t hash_next;
    uint16_t heap_pos;
    uint8_t player;
    uint8_t action_from;
    uint8_t action_dir;
    uint8_t blast_centers[SOKOBAN_MAX_BOMBS];
    uint8_t action_blast;
    uint8_t flags;
} sk_labeled_node_t;

typedef char sk_labeled_node_size_must_not_exceed_32[(sizeof(sk_labeled_node_t) <= 32u) ? 1 : -1];

typedef struct
{
    sk_labeled_node_t *nodes;
    uint16_t *heap;
    uint16_t *hash;
    uint16_t node_count;
    uint16_t heap_size;
    uint16_t expanded;
} sk_labeled_search_t;

typedef struct
{
    uint64_t wall[SK_BITSET_WORDS];
    uint8_t start_player;
    uint8_t box_count;
    uint8_t goal_count;
    uint8_t box_cells[SOKOBAN_MAX_BOXES];
    uint8_t goal_cells[SOKOBAN_MAX_BOXES];
    uint8_t box_goal_cells[SOKOBAN_MAX_BOXES];
    uint8_t bomb_indices[SOKOBAN_MAX_BOMBS];
    uint8_t bomb_count;
    uint16_t normal_box_mask;
    uint16_t all_goal_mask;
} sk_labeled_problem_t;

typedef union
{
    struct
    {
        sk_node_t nodes[SK_NODE_CAPACITY];
        uint16_t heap[SK_NODE_CAPACITY];
        uint16_t hash[SK_HASH_SIZE];
    } legacy;
    struct
    {
        sk_labeled_node_t nodes[SK_LABELED_NODE_CAPACITY];
        uint16_t heap[SK_LABELED_NODE_CAPACITY];
        uint16_t hash[SK_LABELED_HASH_SIZE];
    } labeled;
} sk_search_buffers_t;

static sk_problem_t s_problem;
static sk_labeled_problem_t s_labeled_problem;
static uint8_t s_lookup_ready = 0u;
static int16_t s_neighbor[SK_CELL_COUNT][4];
static const char s_move_char[4] = {'U', 'D', 'L', 'R'};

/* Legacy and labeled searches never run concurrently, so they share OCRAM. */
static AT_OCRAM_SECTION_ALIGN(sk_search_buffers_t s_search_buffers, 32);
#define s_forward_nodes (s_search_buffers.legacy.nodes)
#define s_forward_heap  (s_search_buffers.legacy.heap)
#define s_forward_hash  (s_search_buffers.legacy.hash)

static uint8_t s_bfs_queue[SK_CELL_COUNT];
static uint8_t s_bfs_parent[SK_CELL_COUNT];
static uint8_t s_bfs_parent_dir[SK_CELL_COUNT];
static uint8_t s_bfs_visited[SK_CELL_COUNT];
static uint8_t s_box_cells[SOKOBAN_MAX_BOXES];
static uint8_t s_goal_cells[SOKOBAN_MAX_BOXES];
static uint16_t s_forward_chain[SK_NODE_CAPACITY];

static AT_OCRAM_SECTION_ALIGN(uint16_t s_single_queue[SK_SINGLE_STATE_COUNT], 32);
static AT_OCRAM_SECTION_ALIGN(uint8_t s_single_trace[SK_SINGLE_STATE_COUNT], 32);
static char s_single_reverse_buf[SOKOBAN_MAX_MOVES];
static uint8_t s_single_reverse_push[SOKOBAN_MAX_MOVES];
static sokoban_solution_t s_decompose_candidate_solution;
static sokoban_solution_t s_decompose_best_solution;
static sokoban_solution_t s_bomb_walk_solution;
static sokoban_solution_t s_inspection_candidate_solution;
static sokoban_solution_t s_inspection_best_solution;

/* Reverse a direction id: up<->down, left<->right. */
static uint8_t sk_opposite_dir(uint8_t dir)
{
    return (uint8_t)(dir ^ 1u);
}

static void sk_solution_reset(sokoban_solution_t *solution)
{
    uint8_t idx;
    (void)memset(solution, 0, sizeof(*solution));
    for(idx = 0u; idx < SOKOBAN_MAX_BOMBS; idx++)
    {
        solution->blast_move_indices[idx] = SOKOBAN_INVALID_MOVE_INDEX;
        solution->blast_rows[idx] = SOKOBAN_INVALID_CELL;
        solution->blast_cols[idx] = SOKOBAN_INVALID_CELL;
    }
}

/* Basic bitset helpers used by walls, goals and boxes. */
static void sk_zero_bits(uint64_t bits[SK_BITSET_WORDS])
{
    uint8_t i;
    for(i = 0u; i < SK_BITSET_WORDS; i++)
    {
        bits[i] = 0u;
    }
}

static void sk_copy_bits(uint64_t dst[SK_BITSET_WORDS], const uint64_t src[SK_BITSET_WORDS])
{
    uint8_t i;
    for(i = 0u; i < SK_BITSET_WORDS; i++)
    {
        dst[i] = src[i];
    }
}

static uint8_t sk_test_bit(const uint64_t bits[SK_BITSET_WORDS], uint8_t cell)
{
    return (uint8_t)((bits[cell >> 6] >> (cell & 63u)) & 1u);
}

static void sk_set_bit(uint64_t bits[SK_BITSET_WORDS], uint8_t cell)
{
    bits[cell >> 6] |= (uint64_t)1u << (cell & 63u);
}

static void sk_clear_bit(uint64_t bits[SK_BITSET_WORDS], uint8_t cell)
{
    bits[cell >> 6] &= ~((uint64_t)1u << (cell & 63u));
}

static void sk_move_bit(uint64_t bits[SK_BITSET_WORDS], uint8_t from_cell, uint8_t to_cell)
{
    sk_clear_bit(bits, from_cell);
    sk_set_bit(bits, to_cell);
}

static uint8_t sk_bits_equal(const uint64_t lhs[SK_BITSET_WORDS], const uint64_t rhs[SK_BITSET_WORDS])
{
    uint8_t i;
    for(i = 0u; i < SK_BITSET_WORDS; i++)
    {
        if(lhs[i] != rhs[i])
        {
            return 0u;
        }
    }
    return 1u;
}

/* A state is solved when no box remains on board. */
static uint8_t sk_is_goal_state(const uint64_t boxes[SK_BITSET_WORDS])
{
    uint8_t i;
    for(i = 0u; i < SK_BITSET_WORDS; i++)
    {
        if(boxes[i] != 0u)
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t sk_is_wall(uint8_t cell)
{
    if(cell == SK_INVALID_CELL)
    {
        return 1u;
    }
    return sk_test_bit(s_problem.wall, cell);
}

/* A free cell means: inside map, not a wall, and not occupied by a box. */
static uint8_t sk_is_free_cell(const uint64_t boxes[SK_BITSET_WORDS], uint8_t cell)
{
    if(cell == SK_INVALID_CELL)
    {
        return 0u;
    }
    if(sk_test_bit(s_problem.wall, cell))
    {
        return 0u;
    }
    if(sk_test_bit(boxes, cell))
    {
        return 0u;
    }
    return 1u;
}

/*
 * Find all cells the player can currently walk to without moving any box.
 * out_min_cell is used as the canonical player position for state deduplication.
 */
static uint8_t sk_collect_reachable(const uint64_t boxes[SK_BITSET_WORDS],
                                    uint8_t start_cell,
                                    uint8_t *out_cells,
                                    uint8_t *out_count,
                                    uint8_t *out_min_cell)
{
    uint8_t head = 0u;
    uint8_t tail = 0u;
    uint8_t min_cell = start_cell;

    if(!sk_is_free_cell(boxes, start_cell))
    {
        return 0u;
    }

    (void)memset(s_bfs_visited, 0, sizeof(s_bfs_visited));
    s_bfs_queue[tail++] = start_cell;
    s_bfs_visited[start_cell] = 1u;
    *out_count = 0u;

    while(head < tail)
    {
        uint8_t cell = s_bfs_queue[head++];
        uint8_t dir;

        if(cell < min_cell)
        {
            min_cell = cell;
        }

        if(out_cells != 0)
        {
            out_cells[*out_count] = cell;
        }
        (*out_count)++;

        for(dir = 0u; dir < 4u; dir++)
        {
            int16_t next = s_neighbor[cell][dir];
            if((next >= 0) && (s_bfs_visited[(uint8_t)next] == 0u) && sk_is_free_cell(boxes, (uint8_t)next))
            {
                s_bfs_visited[(uint8_t)next] = 1u;
                s_bfs_queue[tail++] = (uint8_t)next;
            }
        }
    }

    if(out_min_cell != 0)
    {
        *out_min_cell = min_cell;
    }
    return 1u;
}

/* Rebuild a pure walking path between two cells under the current box layout. */
static uint8_t sk_find_player_path(const uint64_t boxes[SK_BITSET_WORDS],
                                   uint8_t start_cell,
                                   uint8_t target_cell,
                                   char *out_moves,
                                   uint16_t *io_move_count)
{
    uint8_t head = 0u;
    uint8_t tail = 0u;
    uint8_t cell;
    uint16_t current_count = *io_move_count;
    char reverse_buf[SK_CELL_COUNT];
    uint16_t reverse_len = 0u;

    if(start_cell == target_cell)
    {
        return 1u;
    }

    (void)memset(s_bfs_visited, 0, sizeof(s_bfs_visited));
    (void)memset(s_bfs_parent, SK_INVALID_CELL, sizeof(s_bfs_parent));

    s_bfs_queue[tail++] = start_cell;
    s_bfs_visited[start_cell] = 1u;

    while(head < tail)
    {
        cell = s_bfs_queue[head++];
        if(cell == target_cell)
        {
            break;
        }

        {
            uint8_t dir;
            for(dir = 0u; dir < 4u; dir++)
            {
                int16_t next = s_neighbor[cell][dir];
                if((next >= 0) && (s_bfs_visited[(uint8_t)next] == 0u) && sk_is_free_cell(boxes, (uint8_t)next))
                {
                    s_bfs_visited[(uint8_t)next] = 1u;
                    s_bfs_parent[(uint8_t)next] = cell;
                    s_bfs_parent_dir[(uint8_t)next] = dir;
                    s_bfs_queue[tail++] = (uint8_t)next;
                }
            }
        }
    }

    if(s_bfs_visited[target_cell] == 0u)
    {
        return 0u;
    }

    cell = target_cell;
    while(cell != start_cell)
    {
        reverse_buf[reverse_len++] = s_move_char[s_bfs_parent_dir[cell]];
        cell = s_bfs_parent[cell];
    }

    if((uint32_t)current_count + reverse_len > SOKOBAN_MAX_MOVES)
    {
        return 0u;
    }

    while(reverse_len > 0u)
    {
        out_moves[current_count++] = reverse_buf[--reverse_len];
    }

    *io_move_count = current_count;
    return 1u;
}

/* Enumerate all occupied cells in one bitset and return count. */
static uint8_t sk_collect_cells(const uint64_t bits[SK_BITSET_WORDS], uint8_t *out_cells)
{
    uint8_t cell;
    uint8_t count = 0u;
    for(cell = 0u; cell < SK_CELL_COUNT; cell++)
    {
        if(sk_test_bit(bits, cell))
        {
            if(count < SOKOBAN_MAX_BOXES)
            {
                out_cells[count++] = cell;
            }
        }
    }
    return count;
}

static uint8_t sk_manhattan(uint8_t lhs, uint8_t rhs)
{
    uint8_t ly = (uint8_t)(lhs / SOKOBAN_MAP_WIDTH);
    uint8_t lx = (uint8_t)(lhs % SOKOBAN_MAP_WIDTH);
    uint8_t ry = (uint8_t)(rhs / SOKOBAN_MAP_WIDTH);
    uint8_t rx = (uint8_t)(rhs % SOKOBAN_MAP_WIDTH);
    return (uint8_t)((lx > rx ? lx - rx : rx - lx) + (ly > ry ? ly - ry : ry - ly));
}

static uint8_t sk_cell_from_row_col(uint8_t row, uint8_t col, uint8_t *out_cell)
{
    if((row >= SOKOBAN_MAP_HEIGHT) || (col >= SOKOBAN_MAP_WIDTH) || (out_cell == 0))
    {
        return 0u;
    }

    *out_cell = (uint8_t)(row * SOKOBAN_MAP_WIDTH + col);
    return 1u;
}

static uint16_t sk_single_state_id(uint8_t player, uint8_t box)
{
    return (uint16_t)((uint16_t)player * (uint16_t)SK_CELL_COUNT + box);
}

static uint8_t sk_cell_is_map_wall_token(char ch)
{
    return (uint8_t)('#' == ch);
}

static uint8_t sk_cell_is_map_box_token(char ch)
{
    return (uint8_t)(('$' == ch) || ('*' == ch));
}

static uint8_t sk_map_cell_walkable(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                    uint8_t cell,
                                    uint8_t opened_wall)
{
    uint8_t row;
    uint8_t col;
    char ch;

    if(cell == SK_INVALID_CELL)
    {
        return 0u;
    }

    if(cell == opened_wall)
    {
        return 1u;
    }

    row = (uint8_t)(cell / SOKOBAN_MAP_WIDTH);
    col = (uint8_t)(cell % SOKOBAN_MAP_WIDTH);
    ch = map[row][col];

    if(sk_cell_is_map_wall_token(ch) || sk_cell_is_map_box_token(ch))
    {
        return 0u;
    }

    return 1u;
}

static uint8_t sk_append_reconstructed_path(uint16_t start_state,
                                            uint16_t end_state,
                                            char *out_moves,
                                            uint16_t *io_move_count,
                                            uint16_t *io_push_count)
{
    uint16_t reverse_len = 0u;
    uint16_t state = end_state;
    uint16_t current_count = *io_move_count;
    uint16_t current_push_count = (io_push_count != 0) ? *io_push_count : 0u;

    while(state != start_state)
    {
        uint8_t trace;
        uint8_t dir;
        uint8_t player;
        uint8_t box;
        int16_t parent_player;

        if(state >= SK_SINGLE_STATE_COUNT)
        {
            return 0u;
        }
        trace = s_single_trace[state];
        if((trace == SK_TRACE_UNVISITED) || (trace == SK_TRACE_START))
        {
            return 0u;
        }
        if(reverse_len >= SOKOBAN_MAX_MOVES)
        {
            return 0u;
        }

        dir = (uint8_t)(trace & 0x03u);
        player = (uint8_t)(state / SK_CELL_COUNT);
        box = (uint8_t)(state % SK_CELL_COUNT);
        s_single_reverse_buf[reverse_len++] = s_move_char[dir];
        s_single_reverse_push[reverse_len - 1u] = (uint8_t)((trace & SK_TRACE_PUSH) != 0u);

        if((trace & SK_TRACE_PUSH) != 0u)
        {
            box = player;
            parent_player = s_neighbor[box][sk_opposite_dir(dir)];
        }
        else
        {
            parent_player = s_neighbor[player][sk_opposite_dir(dir)];
        }
        if(parent_player < 0)
        {
            return 0u;
        }
        state = sk_single_state_id((uint8_t)parent_player, box);
    }

    if((uint32_t)current_count + reverse_len > SOKOBAN_MAX_MOVES)
    {
        return 0u;
    }

    while(reverse_len > 0u)
    {
        reverse_len--;
        if(s_single_reverse_push[reverse_len] && (io_push_count != 0))
        {
            current_push_count++;
        }
        out_moves[current_count++] = s_single_reverse_buf[reverse_len];
    }

    *io_move_count = current_count;
    if(io_push_count != 0)
    {
        *io_push_count = current_push_count;
    }
    return 1u;
}

/* Heuristic: each remaining box estimates distance to nearest remaining goal. */
static uint16_t sk_forward_heuristic(const uint64_t boxes[SK_BITSET_WORDS],
                                     const uint64_t goals[SK_BITSET_WORDS])
{
    uint16_t heuristic_cost = 0u;
    uint8_t box_count;
    uint8_t goal_count;
    uint8_t box_idx;

    box_count = sk_collect_cells(boxes, s_box_cells);
    goal_count = sk_collect_cells(goals, s_goal_cells);

    if(box_count == 0u)
    {
        return 0u;
    }
    if((goal_count == 0u) || (box_count > goal_count))
    {
        return 0x7FFFu;
    }

    for(box_idx = 0u; box_idx < box_count; box_idx++)
    {
        uint8_t goal_idx;
        uint8_t best_dist = 0xFFu;
        for(goal_idx = 0u; goal_idx < goal_count; goal_idx++)
        {
            uint8_t d = sk_manhattan(s_box_cells[box_idx], s_goal_cells[goal_idx]);
            if(d < best_dist)
            {
                best_dist = d;
            }
        }
        heuristic_cost = (uint16_t)(heuristic_cost + best_dist);
    }
    return heuristic_cost;
}

/* Hash = quick lookup for checking whether a state was already visited. */
static uint32_t sk_hash_state(const uint64_t boxes[SK_BITSET_WORDS],
                              const uint64_t goals[SK_BITSET_WORDS],
                              uint8_t player)
{
    uint32_t h;
    uint32_t a = (uint32_t)(boxes[0] ^ (boxes[0] >> 32));
    uint32_t b = (uint32_t)(boxes[1] ^ (boxes[1] >> 32));
    uint32_t c = (uint32_t)(boxes[2] ^ (boxes[2] >> 32));
    uint32_t d = (uint32_t)(goals[0] ^ (goals[0] >> 32));
    uint32_t e = (uint32_t)(goals[1] ^ (goals[1] >> 32));
    uint32_t f = (uint32_t)(goals[2] ^ (goals[2] >> 32));
    h = a * 2654435761u;
    h ^= b * 2246822519u;
    h ^= c * 3266489917u;
    h ^= d * 668265263u;
    h ^= e * 374761393u;
    h ^= f * 1274126177u;
    h ^= (uint32_t)player * 668265263u;
    return h % SK_HASH_SIZE;
}

/* Binary heap helpers implement the A* open list priority queue. */
static uint8_t sk_heap_less(const sk_search_t *ctx, uint16_t lhs_idx, uint16_t rhs_idx)
{
    const sk_node_t *lhs = &ctx->nodes[lhs_idx];
    const sk_node_t *rhs = &ctx->nodes[rhs_idx];
    uint16_t lhs_f = (uint16_t)(lhs->g + lhs->h);
    uint16_t rhs_f = (uint16_t)(rhs->g + rhs->h);

    if(lhs_f != rhs_f)
    {
        return (uint8_t)(lhs_f < rhs_f);
    }
    if(lhs->h != rhs->h)
    {
        return (uint8_t)(lhs->h < rhs->h);
    }
    return (uint8_t)(lhs->g > rhs->g);
}

static void sk_heap_swap(sk_search_t *ctx, uint16_t a_pos, uint16_t b_pos)
{
    uint16_t tmp = ctx->heap[a_pos];
    ctx->heap[a_pos] = ctx->heap[b_pos];
    ctx->heap[b_pos] = tmp;
    ctx->nodes[ctx->heap[a_pos]].heap_pos = a_pos;
    ctx->nodes[ctx->heap[b_pos]].heap_pos = b_pos;
}

static void sk_heap_up(sk_search_t *ctx, uint16_t pos)
{
    while(pos > 0u)
    {
        uint16_t parent = (uint16_t)((pos - 1u) >> 1);
        if(!sk_heap_less(ctx, ctx->heap[pos], ctx->heap[parent]))
        {
            break;
        }
        sk_heap_swap(ctx, pos, parent);
        pos = parent;
    }
}

static void sk_heap_down(sk_search_t *ctx, uint16_t pos)
{
    while(1)
    {
        uint16_t left = (uint16_t)(pos * 2u + 1u);
        uint16_t right = (uint16_t)(left + 1u);
        uint16_t best = pos;

        if((left < ctx->heap_size) && sk_heap_less(ctx, ctx->heap[left], ctx->heap[best]))
        {
            best = left;
        }
        if((right < ctx->heap_size) && sk_heap_less(ctx, ctx->heap[right], ctx->heap[best]))
        {
            best = right;
        }
        if(best == pos)
        {
            break;
        }
        sk_heap_swap(ctx, pos, best);
        pos = best;
    }
}

static void sk_heap_push(sk_search_t *ctx, uint16_t node_idx)
{
    ctx->heap[ctx->heap_size] = node_idx;
    ctx->nodes[node_idx].heap_pos = ctx->heap_size;
    ctx->heap_size++;
    sk_heap_up(ctx, (uint16_t)(ctx->heap_size - 1u));
}

static uint16_t sk_heap_pop(sk_search_t *ctx)
{
    uint16_t result = ctx->heap[0];
    ctx->nodes[result].heap_pos = SK_INVALID_NODE;
    ctx->heap_size--;
    if(ctx->heap_size > 0u)
    {
        ctx->heap[0] = ctx->heap[ctx->heap_size];
        ctx->nodes[ctx->heap[0]].heap_pos = 0u;
        sk_heap_down(ctx, 0u);
    }
    return result;
}

static uint16_t sk_heap_min_f(const sk_search_t *ctx)
{
    if(ctx->heap_size == 0u)
    {
        return 0xFFFFu;
    }
    return (uint16_t)(ctx->nodes[ctx->heap[0]].g + ctx->nodes[ctx->heap[0]].h);
}

static uint16_t sk_find_state(const sk_search_t *ctx,
                              const uint64_t boxes[SK_BITSET_WORDS],
                              const uint64_t goals[SK_BITSET_WORDS],
                              uint8_t player)
{
    uint32_t bucket = sk_hash_state(boxes, goals, player);
    uint16_t node_idx = ctx->hash[bucket];

    while(node_idx != SK_INVALID_NODE)
    {
        const sk_node_t *node = &ctx->nodes[node_idx];
        if((node->player == player) && sk_bits_equal(node->boxes, boxes) && sk_bits_equal(node->goals, goals))
        {
            return node_idx;
        }
        node_idx = node->hash_next;
    }
    return SK_INVALID_NODE;
}

static void sk_insert_hash(sk_search_t *ctx, uint16_t node_idx)
{
    uint32_t bucket = sk_hash_state(ctx->nodes[node_idx].boxes, ctx->nodes[node_idx].goals, ctx->nodes[node_idx].player);
    ctx->nodes[node_idx].hash_next = ctx->hash[bucket];
    ctx->hash[bucket] = node_idx;
}

static void sk_search_reset(sk_search_t *ctx, sk_node_t *nodes, uint16_t *heap, uint16_t *hash)
{
    uint16_t i;
    ctx->nodes = nodes;
    ctx->heap = heap;
    ctx->hash = hash;
    ctx->node_count = 0u;
    ctx->heap_size = 0u;
    ctx->expanded = 0u;
    for(i = 0u; i < SK_HASH_SIZE; i++)
    {
        ctx->hash[i] = SK_INVALID_NODE;
    }
}

static uint8_t sk_add_or_update(sk_search_t *ctx,
                                const uint64_t boxes[SK_BITSET_WORDS],
                                const uint64_t goals[SK_BITSET_WORDS],
                                uint8_t player,
                                uint16_t g,
                                uint16_t h,
                                uint16_t parent,
                                uint8_t action_from,
                                uint8_t action_dir,
                                uint16_t *out_node_idx)
{
    uint16_t node_idx = sk_find_state(ctx, boxes, goals, player);

    if(node_idx != SK_INVALID_NODE)
    {
        sk_node_t *node = &ctx->nodes[node_idx];
        if(g < node->g)
        {
            node->g = g;
            node->h = h;
            node->parent = parent;
            node->action_from = action_from;
            node->action_dir = action_dir;
            node->flags &= (uint8_t)~SK_FLAG_CLOSED;
            if(node->heap_pos == SK_INVALID_NODE)
            {
                sk_heap_push(ctx, node_idx);
            }
            else
            {
                sk_heap_up(ctx, node->heap_pos);
            }
        }
        *out_node_idx = node_idx;
        return 1u;
    }

    if(ctx->node_count >= SK_NODE_CAPACITY)
    {
        return 0u;
    }

    node_idx = ctx->node_count++;
    sk_copy_bits(ctx->nodes[node_idx].boxes, boxes);
    sk_copy_bits(ctx->nodes[node_idx].goals, goals);
    ctx->nodes[node_idx].player = player;
    ctx->nodes[node_idx].g = g;
    ctx->nodes[node_idx].h = h;
    ctx->nodes[node_idx].parent = parent;
    ctx->nodes[node_idx].action_from = action_from;
    ctx->nodes[node_idx].action_dir = action_dir;
    ctx->nodes[node_idx].flags = 0u;
    ctx->nodes[node_idx].heap_pos = SK_INVALID_NODE;
    ctx->nodes[node_idx].hash_next = SK_INVALID_NODE;
    sk_insert_hash(ctx, node_idx);
    sk_heap_push(ctx, node_idx);
    *out_node_idx = node_idx;
    return 1u;
}

/* Build the neighbor table once, then reuse it everywhere. */
static void sk_init_lookup(void)
{
    uint8_t y;
    uint8_t x;

    if(s_lookup_ready)
    {
        return;
    }

    for(y = 0u; y < SOKOBAN_MAP_HEIGHT; y++)
    {
        for(x = 0u; x < SOKOBAN_MAP_WIDTH; x++)
        {
            uint8_t cell = (uint8_t)(y * SOKOBAN_MAP_WIDTH + x);
            s_neighbor[cell][SK_DIR_UP] = (y > 0u) ? (int16_t)(cell - SOKOBAN_MAP_WIDTH) : -1;
            s_neighbor[cell][SK_DIR_DOWN] = (y + 1u < SOKOBAN_MAP_HEIGHT) ? (int16_t)(cell + SOKOBAN_MAP_WIDTH) : -1;
            s_neighbor[cell][SK_DIR_LEFT] = (x > 0u) ? (int16_t)(cell - 1u) : -1;
            s_neighbor[cell][SK_DIR_RIGHT] = (x + 1u < SOKOBAN_MAP_WIDTH) ? (int16_t)(cell + 1u) : -1;
        }
    }

    s_lookup_ready = 1u;
}

/*
 * Mark dead squares:
 * if a box reaches such a cell, it can never be pushed onto any goal later.
 */
static void sk_build_dead_squares(void)
{
    uint8_t queue[SK_CELL_COUNT];
    uint8_t visited[SK_CELL_COUNT];
    uint8_t goal_idx;

    sk_zero_bits(s_problem.dead_square);
    (void)memset(visited, 0, sizeof(visited));

    for(goal_idx = 0u; goal_idx < s_problem.goal_count; goal_idx++)
    {
        uint8_t head = 0u;
        uint8_t tail = 0u;

        queue[tail++] = s_problem.goal_cells[goal_idx];
        visited[s_problem.goal_cells[goal_idx]] = 1u;

        while(head < tail)
        {
            uint8_t cell = queue[head++];
            uint8_t dir;

            for(dir = 0u; dir < 4u; dir++)
            {
                int16_t prev = s_neighbor[cell][sk_opposite_dir(dir)];
                int16_t stand = (prev >= 0) ? s_neighbor[(uint8_t)prev][sk_opposite_dir(dir)] : -1;

                if((prev >= 0) && (stand >= 0) && !sk_is_wall((uint8_t)prev) && !sk_is_wall((uint8_t)stand) &&
                   (visited[(uint8_t)prev] == 0u))
                {
                    visited[(uint8_t)prev] = 1u;
                    queue[tail++] = (uint8_t)prev;
                }
            }
        }
    }

    {
        uint8_t cell;
        for(cell = 0u; cell < SK_CELL_COUNT; cell++)
        {
            if(!sk_is_wall(cell) && (visited[cell] == 0u))
            {
                sk_set_bit(s_problem.dead_square, cell);
            }
        }
    }
}

/* Detect one simple deadlock pattern: two boxes stuck against a wall line. */
static uint8_t sk_pair_wall_deadlock(const uint64_t boxes[SK_BITSET_WORDS],
                                     const uint64_t goals[SK_BITSET_WORDS],
                                     uint8_t first,
                                     uint8_t second,
                                     uint8_t side_a,
                                     uint8_t side_b)
{
    if(!sk_test_bit(boxes, first))
    {
        return 0u;
    }
    if(!sk_test_bit(boxes, second))
    {
        return 0u;
    }
    if(sk_test_bit(goals, first) || sk_test_bit(goals, second))
    {
        return 0u;
    }
    return (uint8_t)(sk_is_wall(side_a) && sk_is_wall(side_b));
}

static uint8_t sk_has_adjacent_deadlock(const uint64_t boxes[SK_BITSET_WORDS],
                                        const uint64_t goals[SK_BITSET_WORDS],
                                        uint8_t moved_cell)
{
    int16_t left = s_neighbor[moved_cell][SK_DIR_LEFT];
    int16_t right = s_neighbor[moved_cell][SK_DIR_RIGHT];
    int16_t up = s_neighbor[moved_cell][SK_DIR_UP];
    int16_t down = s_neighbor[moved_cell][SK_DIR_DOWN];

    if((left >= 0) &&
       (sk_pair_wall_deadlock(boxes, goals, (uint8_t)left, moved_cell,
                              (up >= 0) ? (uint8_t)s_neighbor[(uint8_t)left][SK_DIR_UP] : SK_INVALID_CELL,
                              (uint8_t)up) ||
        sk_pair_wall_deadlock(boxes, goals, (uint8_t)left, moved_cell,
                              (down >= 0) ? (uint8_t)s_neighbor[(uint8_t)left][SK_DIR_DOWN] : SK_INVALID_CELL,
                              (uint8_t)down)))
    {
        return 1u;
    }

    if((right >= 0) &&
       (sk_pair_wall_deadlock(boxes, goals, moved_cell, (uint8_t)right,
                              (uint8_t)up,
                              (up >= 0) ? (uint8_t)s_neighbor[(uint8_t)right][SK_DIR_UP] : SK_INVALID_CELL) ||
        sk_pair_wall_deadlock(boxes, goals, moved_cell, (uint8_t)right,
                              (uint8_t)down,
                              (down >= 0) ? (uint8_t)s_neighbor[(uint8_t)right][SK_DIR_DOWN] : SK_INVALID_CELL)))
    {
        return 1u;
    }

    if((up >= 0) &&
       (sk_pair_wall_deadlock(boxes, goals, (uint8_t)up, moved_cell,
                              (left >= 0) ? (uint8_t)s_neighbor[(uint8_t)up][SK_DIR_LEFT] : SK_INVALID_CELL,
                              (uint8_t)left) ||
        sk_pair_wall_deadlock(boxes, goals, (uint8_t)up, moved_cell,
                              (right >= 0) ? (uint8_t)s_neighbor[(uint8_t)up][SK_DIR_RIGHT] : SK_INVALID_CELL,
                              (uint8_t)right)))
    {
        return 1u;
    }

    if((down >= 0) &&
       (sk_pair_wall_deadlock(boxes, goals, moved_cell, (uint8_t)down,
                              (uint8_t)left,
                              (left >= 0) ? (uint8_t)s_neighbor[(uint8_t)down][SK_DIR_LEFT] : SK_INVALID_CELL) ||
        sk_pair_wall_deadlock(boxes, goals, moved_cell, (uint8_t)down,
                              (uint8_t)right,
                              (right >= 0) ? (uint8_t)s_neighbor[(uint8_t)down][SK_DIR_RIGHT] : SK_INVALID_CELL)))
    {
        return 1u;
    }

    return 0u;
}

/* Deadlock pruning that runs right after generating a new box position. */
static uint8_t sk_is_deadlock_after_move(const uint64_t boxes[SK_BITSET_WORDS],
                                         const uint64_t goals[SK_BITSET_WORDS],
                                         uint8_t moved_cell)
{
    if(!sk_test_bit(goals, moved_cell) && sk_test_bit(s_problem.dead_square, moved_cell))
    {
        return 1u;
    }
    if(sk_has_adjacent_deadlock(boxes, goals, moved_cell))
    {
        return 1u;
    }
    return 0u;
}

static sokoban_status_t sk_add_goal_cell(uint8_t cell)
{
    if(s_problem.goal_count >= SOKOBAN_MAX_BOXES)
    {
        return SOKOBAN_STATUS_TOO_MANY_BOXES;
    }

    sk_set_bit(s_problem.goal, cell);
    s_problem.goal_cells[s_problem.goal_count++] = cell;
    return SOKOBAN_STATUS_OK;
}

static sokoban_status_t sk_add_start_box_cell(uint8_t cell)
{
    if(s_problem.box_count >= SOKOBAN_MAX_BOXES)
    {
        return SOKOBAN_STATUS_TOO_MANY_BOXES;
    }

    sk_set_bit(s_problem.start_boxes, cell);
    s_problem.start_box_cells[s_problem.box_count++] = cell;
    return SOKOBAN_STATUS_OK;
}

/* Parse the text map into bitsets and helper arrays used by the solver. */
static sokoban_status_t sk_parse_map(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE])
{
    uint8_t y;
    uint8_t x;
    uint8_t player_found = 0u;

    (void)memset(&s_problem, 0, sizeof(s_problem));
    sk_zero_bits(s_problem.wall);
    sk_zero_bits(s_problem.goal);
    sk_zero_bits(s_problem.start_boxes);

    for(y = 0u; y < SOKOBAN_MAP_HEIGHT; y++)
    {
        for(x = 0u; x < SOKOBAN_MAP_WIDTH; x++)
        {
            char ch = map[y][x];
            uint8_t cell = (uint8_t)(y * SOKOBAN_MAP_WIDTH + x);

            switch(ch)
            {
                case '#':
                {
                    sk_set_bit(s_problem.wall, cell);
                } break;

                case '-':
                {
                } break;

                case '.':
                {
                    sokoban_status_t add_goal_status = sk_add_goal_cell(cell);
                    if(add_goal_status != SOKOBAN_STATUS_OK)
                    {
                        return add_goal_status;
                    }
                } break;

                case 'G':
                {
                    sokoban_status_t add_goal_status = sk_add_goal_cell(cell);
                    if(add_goal_status != SOKOBAN_STATUS_OK)
                    {
                        return add_goal_status;
                    }
                } break;

                case '$':
                {
                    sokoban_status_t add_box_status = sk_add_start_box_cell(cell);
                    if(add_box_status != SOKOBAN_STATUS_OK)
                    {
                        return add_box_status;
                    }
                } break;

                case '*':
                {
                    return SOKOBAN_STATUS_INVALID_MAP;
                } break;

                case '@':
                {
                    if(player_found)
                    {
                        return SOKOBAN_STATUS_INVALID_MAP;
                    }
                    s_problem.start_player = cell;
                    player_found = 1u;
                } break;

                case '+':
                {
                    if(player_found || (s_problem.goal_count >= SOKOBAN_MAX_BOXES))
                    {
                        return SOKOBAN_STATUS_INVALID_MAP;
                    }
                    s_problem.start_player = cell;
                    player_found = 1u;
                    sk_set_bit(s_problem.goal, cell);
                    s_problem.goal_cells[s_problem.goal_count++] = cell;
                } break;

                default:
                {
                    return SOKOBAN_STATUS_INVALID_MAP;
                } break;
            }
        }
    }

    if((player_found == 0u) || (s_problem.box_count == 0u) || (s_problem.goal_count == 0u))
    {
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    if(s_problem.box_count != s_problem.goal_count)
    {
        return SOKOBAN_STATUS_BOX_GOAL_MISMATCH;
    }

    sk_build_dead_squares();

    return SOKOBAN_STATUS_OK;
}

static uint8_t sk_remaining_box_blocks_cell(const uint8_t remaining_boxes[SOKOBAN_MAX_BOXES],
                                            uint8_t active_box_idx,
                                            uint8_t cell)
{
    uint8_t idx;

    for(idx = 0u; idx < s_problem.box_count; idx++)
    {
        if((remaining_boxes[idx] != 0u) &&
           (idx != active_box_idx) &&
           (s_problem.start_box_cells[idx] == cell))
        {
            return 1u;
        }
    }

    return 0u;
}

static uint8_t sk_single_cell_free(const uint8_t remaining_boxes[SOKOBAN_MAX_BOXES],
                                   uint8_t active_box_idx,
                                   uint8_t active_box_cell,
                                   uint8_t cell)
{
    if(cell == SK_INVALID_CELL)
    {
        return 0u;
    }

    if(sk_is_wall(cell))
    {
        return 0u;
    }

    if(cell == active_box_cell)
    {
        return 0u;
    }

    if(sk_remaining_box_blocks_cell(remaining_boxes, active_box_idx, cell))
    {
        return 0u;
    }

    return 1u;
}

static void sk_reset_single_bfs(void)
{
    (void)memset(s_single_trace, SK_TRACE_UNVISITED, sizeof(s_single_trace));
}

static sokoban_status_t sk_solve_single_box(uint8_t start_player,
                                            uint8_t start_box,
                                            uint8_t target_goal,
                                            const uint8_t remaining_boxes[SOKOBAN_MAX_BOXES],
                                            uint8_t active_box_idx,
                                            sokoban_solution_t *solution,
                                            uint8_t *out_end_player)
{
    uint16_t head = 0u;
    uint16_t tail = 0u;
    uint16_t start_state;
    uint16_t expanded = 0u;

    sk_solution_reset(solution);

    if(start_box == target_goal)
    {
        solution->solved = 1u;
        if(out_end_player != 0)
        {
            *out_end_player = start_player;
        }
        return SOKOBAN_STATUS_OK;
    }

    if(!sk_single_cell_free(remaining_boxes, active_box_idx, SK_INVALID_CELL, start_player))
    {
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    sk_reset_single_bfs();
    start_state = sk_single_state_id(start_player, start_box);
    s_single_queue[tail++] = start_state;
    s_single_trace[start_state] = SK_TRACE_START;

    while(head < tail)
    {
        uint16_t state = s_single_queue[head++];
        uint8_t player = (uint8_t)(state / SK_CELL_COUNT);
        uint8_t box = (uint8_t)(state % SK_CELL_COUNT);
        uint8_t dir;

        expanded++;

        for(dir = 0u; dir < 4u; dir++)
        {
            int16_t next_player_i = s_neighbor[player][dir];
            uint8_t next_player;

            if(next_player_i < 0)
            {
                continue;
            }

            next_player = (uint8_t)next_player_i;

            if(next_player == box)
            {
                int16_t next_box_i = s_neighbor[box][dir];
                uint8_t next_box;
                uint16_t child_state;

                if(next_box_i < 0)
                {
                    continue;
                }

                next_box = (uint8_t)next_box_i;
                if(!sk_single_cell_free(remaining_boxes, active_box_idx, SK_INVALID_CELL, next_box))
                {
                    continue;
                }

                child_state = sk_single_state_id(box, next_box);
                if(s_single_trace[child_state] != SK_TRACE_UNVISITED)
                {
                    continue;
                }

                s_single_trace[child_state] = (uint8_t)(dir | SK_TRACE_PUSH);

                if(next_box == target_goal)
                {
                    uint16_t move_count = 0u;
                    uint16_t push_count = 0u;
                    if(!sk_append_reconstructed_path(start_state,
                                                     child_state,
                                                     solution->move_seq,
                                                     &move_count,
                                                     &push_count))
                    {
                        return SOKOBAN_STATUS_PATH_OVERFLOW;
                    }

                    solution->move_seq[move_count] = '\0';
                    solution->move_count = move_count;
                    solution->push_count = push_count;
                    solution->expanded_forward = expanded;
                    solution->expanded_reverse = 0u;
                    solution->solved = 1u;
                    if(out_end_player != 0)
                    {
                        *out_end_player = box;
                    }
                    return SOKOBAN_STATUS_OK;
                }

                s_single_queue[tail++] = child_state;
            }
            else
            {
                uint16_t child_state;

                if(!sk_single_cell_free(remaining_boxes, active_box_idx, box, next_player))
                {
                    continue;
                }

                child_state = sk_single_state_id(next_player, box);
                if(s_single_trace[child_state] != SK_TRACE_UNVISITED)
                {
                    continue;
                }

                s_single_trace[child_state] = dir;
                s_single_queue[tail++] = child_state;
            }
        }
    }

    solution->expanded_forward = expanded;
    return SOKOBAN_STATUS_UNSOLVABLE;
}

static sokoban_status_t sk_plan_walk_cells(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                           uint8_t start_cell,
                                           uint8_t target_cell,
                                           uint8_t opened_wall,
                                           sokoban_solution_t *solution)
{
    uint16_t head = 0u;
    uint16_t tail = 0u;
    uint16_t start_state;
    uint16_t target_state;
    uint16_t expanded = 0u;

    sk_solution_reset(solution);

    if(!sk_map_cell_walkable(map, start_cell, opened_wall) ||
       !sk_map_cell_walkable(map, target_cell, opened_wall))
    {
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    start_state = sk_single_state_id(start_cell, 0u);
    target_state = sk_single_state_id(target_cell, 0u);

    if(start_cell == target_cell)
    {
        solution->solved = 1u;
        solution->move_seq[0] = '\0';
        return SOKOBAN_STATUS_OK;
    }

    sk_reset_single_bfs();
    s_single_queue[tail++] = start_state;
    s_single_trace[start_state] = SK_TRACE_START;

    while(head < tail)
    {
        uint16_t state = s_single_queue[head++];
        uint8_t player = (uint8_t)(state / SK_CELL_COUNT);
        uint8_t dir;

        expanded++;

        for(dir = 0u; dir < 4u; dir++)
        {
            int16_t next_i = s_neighbor[player][dir];
            uint8_t next;
            uint16_t child_state;

            if(next_i < 0)
            {
                continue;
            }

            next = (uint8_t)next_i;
            if(!sk_map_cell_walkable(map, next, opened_wall))
            {
                continue;
            }

            child_state = sk_single_state_id(next, 0u);
            if(s_single_trace[child_state] != SK_TRACE_UNVISITED)
            {
                continue;
            }

            s_single_trace[child_state] = dir;

            if(child_state == target_state)
            {
                uint16_t move_count = 0u;
                if(!sk_append_reconstructed_path(start_state,
                                                 child_state,
                                                 solution->move_seq,
                                                 &move_count,
                                                 0))
                {
                    return SOKOBAN_STATUS_PATH_OVERFLOW;
                }

                solution->move_seq[move_count] = '\0';
                solution->move_count = move_count;
                solution->push_count = 0u;
                solution->expanded_forward = expanded;
                solution->expanded_reverse = 0u;
                solution->solved = 1u;
                return SOKOBAN_STATUS_OK;
            }

            s_single_queue[tail++] = child_state;
        }
    }

    solution->expanded_forward = expanded;
    return SOKOBAN_STATUS_UNSOLVABLE;
}

static uint8_t sk_append_solution(sokoban_solution_t *dst, const sokoban_solution_t *src)
{
    uint16_t idx;

    if((uint32_t)dst->move_count + src->move_count > SOKOBAN_MAX_MOVES)
    {
        return 0u;
    }

    for(idx = 0u; idx < src->move_count; idx++)
    {
        dst->move_seq[dst->move_count + idx] = src->move_seq[idx];
    }

    dst->move_count = (uint16_t)(dst->move_count + src->move_count);
    dst->push_count = (uint16_t)(dst->push_count + src->push_count);
    dst->expanded_forward = (uint16_t)(dst->expanded_forward + src->expanded_forward);
    dst->move_seq[dst->move_count] = '\0';
    return 1u;
}

static sokoban_status_t sk_init_search(sk_search_t *forward)
{
    uint8_t canonical_player;
    uint8_t count;
    uint16_t node_idx;

    sk_search_reset(forward, s_forward_nodes, s_forward_heap, s_forward_hash);

    if(!sk_collect_reachable(s_problem.start_boxes, s_problem.start_player, 0, &count, &canonical_player))
    {
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    if(!sk_add_or_update(forward,
                         s_problem.start_boxes,
                         s_problem.goal,
                         canonical_player,
                         0u,
                         sk_forward_heuristic(s_problem.start_boxes, s_problem.goal),
                         SK_INVALID_NODE,
                         SK_INVALID_CELL,
                         SK_INVALID_CELL,
                         &node_idx))
    {
        return SOKOBAN_STATUS_NO_MEMORY;
    }

    return SOKOBAN_STATUS_OK;
}

/* Expand one normal-push state from the start side. */
static sokoban_status_t sk_expand_forward(sk_search_t *forward,
                                          uint16_t *best_cost,
                                          uint16_t *best_forward_idx)
{
    uint16_t current_node_idx = sk_heap_pop(forward);
    sk_node_t *current_node = &forward->nodes[current_node_idx];
    uint8_t reachable_cells[SK_CELL_COUNT];
    uint8_t reachable_cell_count = 0u;
    uint8_t reachable_idx;

    current_node->flags |= SK_FLAG_CLOSED;
    forward->expanded++;

    if(sk_is_goal_state(current_node->boxes) && (current_node->g < *best_cost))
    {
        *best_cost = current_node->g;
        *best_forward_idx = current_node_idx;
    }

    if(!sk_collect_reachable(current_node->boxes, current_node->player, reachable_cells, &reachable_cell_count, 0))
    {
        return SOKOBAN_STATUS_OK;
    }

    for(reachable_idx = 0u; reachable_idx < reachable_cell_count; reachable_idx++)
    {
        uint8_t player_cell = reachable_cells[reachable_idx];
        uint8_t push_dir;

        for(push_dir = 0u; push_dir < 4u; push_dir++)
        {
            int16_t box_cell = s_neighbor[player_cell][push_dir];
            int16_t target_cell = (box_cell >= 0) ? s_neighbor[(uint8_t)box_cell][push_dir] : -1;
            if((box_cell < 0) || (target_cell < 0))
            {
                continue;
            }
            if(!sk_test_bit(current_node->boxes, (uint8_t)box_cell) || !sk_is_free_cell(current_node->boxes, (uint8_t)target_cell))
            {
                continue;
            }

            {
                uint64_t next_box_layout[SK_BITSET_WORDS];
                uint64_t next_goals[SK_BITSET_WORDS];
                uint8_t canonical_player;
                uint8_t tmp_count;
                uint16_t child_idx;
                uint8_t solved_on_goal = 0u;

                sk_copy_bits(next_box_layout, current_node->boxes);
                sk_copy_bits(next_goals, current_node->goals);
                sk_move_bit(next_box_layout, (uint8_t)box_cell, (uint8_t)target_cell);

                if(sk_test_bit(next_goals, (uint8_t)target_cell))
                {
                    sk_clear_bit(next_box_layout, (uint8_t)target_cell);
                    sk_clear_bit(next_goals, (uint8_t)target_cell);
                    solved_on_goal = 1u;
                }

                if((solved_on_goal == 0u) &&
                   sk_is_deadlock_after_move(next_box_layout, next_goals, (uint8_t)target_cell))
                {
                    continue;
                }

                if(!sk_collect_reachable(next_box_layout, (uint8_t)box_cell, 0, &tmp_count, &canonical_player))
                {
                    continue;
                }

                if(!sk_add_or_update(forward,
                                     next_box_layout,
                                     next_goals,
                                     canonical_player,
                                     (uint16_t)(current_node->g + 1u),
                                     sk_forward_heuristic(next_box_layout, next_goals),
                                     current_node_idx,
                                     player_cell,
                                     push_dir,
                                     &child_idx))
                {
                    return SOKOBAN_STATUS_NO_MEMORY;
                }

                if(sk_is_goal_state(next_box_layout) && (forward->nodes[child_idx].g < *best_cost))
                {
                    *best_cost = forward->nodes[child_idx].g;
                    *best_forward_idx = child_idx;
                }
            }
        }
    }

    return SOKOBAN_STATUS_OK;
}

/* Rebuild the final U/D/L/R move list from forward path only. */
static sokoban_status_t sk_reconstruct(const sk_search_t *forward,
                                       uint16_t forward_idx,
                                       sokoban_solution_t *solution)
{
    uint64_t boxes[SK_BITSET_WORDS];
    uint64_t goals[SK_BITSET_WORDS];
    uint8_t player = s_problem.start_player;
    uint16_t move_count = 0u;
    uint16_t push_count = 0u;
    uint16_t forward_chain_len = 0u;
    uint16_t current_idx;

    sk_copy_bits(boxes, s_problem.start_boxes);
    sk_copy_bits(goals, s_problem.goal);

    current_idx = forward_idx;
    while(current_idx != SK_INVALID_NODE)
    {
        s_forward_chain[forward_chain_len++] = current_idx;
        current_idx = forward->nodes[current_idx].parent;
    }

    while(forward_chain_len > 1u)
    {
        const sk_node_t *child = &forward->nodes[s_forward_chain[forward_chain_len - 2u]];
        uint8_t box_from;
        uint8_t box_to;

        if(!sk_find_player_path(boxes, player, child->action_from, solution->move_seq, &move_count))
        {
            return SOKOBAN_STATUS_PATH_OVERFLOW;
        }

        if(move_count >= SOKOBAN_MAX_MOVES)
        {
            return SOKOBAN_STATUS_PATH_OVERFLOW;
        }

        solution->move_seq[move_count++] = s_move_char[child->action_dir];
        push_count++;
        box_from = (uint8_t)s_neighbor[child->action_from][child->action_dir];
        box_to = (uint8_t)s_neighbor[box_from][child->action_dir];
        sk_move_bit(boxes, box_from, box_to);
        if(sk_test_bit(goals, box_to))
        {
            sk_clear_bit(boxes, box_to);
            sk_clear_bit(goals, box_to);
        }
        player = box_from;
        forward_chain_len--;
    }

    solution->move_seq[move_count] = '\0';
    solution->move_count = move_count;
    solution->push_count = push_count;
    solution->solved = sk_is_goal_state(boxes);

    return solution->solved ? SOKOBAN_STATUS_OK : SOKOBAN_STATUS_UNSOLVABLE;
}

/* Public entry: greedy decomposed solver using single-box BFS subproblems. */
sokoban_status_t sokoban_solve_decomposed(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                          sokoban_solution_t *solution)
{
    sokoban_status_t status;
    uint8_t remaining_boxes[SOKOBAN_MAX_BOXES];
    uint8_t remaining_goals[SOKOBAN_MAX_BOXES];
    uint8_t remaining_count;
    uint8_t current_player;
    uint8_t idx;

    if((map == 0) || (solution == 0))
    {
        return SOKOBAN_STATUS_INVALID_ARGUMENT;
    }

    sk_solution_reset(solution);
    sk_init_lookup();

    status = sk_parse_map(map);
    if(status != SOKOBAN_STATUS_OK)
    {
        return status;
    }

    for(idx = 0u; idx < SOKOBAN_MAX_BOXES; idx++)
    {
        remaining_boxes[idx] = 0u;
        remaining_goals[idx] = 0u;
    }

    for(idx = 0u; idx < s_problem.box_count; idx++)
    {
        remaining_boxes[idx] = 1u;
        remaining_goals[idx] = 1u;
    }

    current_player = s_problem.start_player;
    remaining_count = s_problem.box_count;

    while(remaining_count > 0u)
    {
        uint8_t candidate_end_player = SK_INVALID_CELL;
        uint8_t best_end_player = SK_INVALID_CELL;
        uint8_t best_box_idx = SK_INVALID_CELL;
        uint8_t best_goal_idx = SK_INVALID_CELL;
        uint16_t best_move_count = 0xFFFFu;
        uint16_t best_push_count = 0xFFFFu;
        uint8_t box_idx;

        (void)memset(&s_decompose_best_solution, 0, sizeof(s_decompose_best_solution));

        for(box_idx = 0u; box_idx < s_problem.box_count; box_idx++)
        {
            uint8_t goal_idx;

            if(remaining_boxes[box_idx] == 0u)
            {
                continue;
            }

            for(goal_idx = 0u; goal_idx < s_problem.goal_count; goal_idx++)
            {
                if(remaining_goals[goal_idx] == 0u)
                {
                    continue;
                }

                status = sk_solve_single_box(current_player,
                                             s_problem.start_box_cells[box_idx],
                                             s_problem.goal_cells[goal_idx],
                                             remaining_boxes,
                                             box_idx,
                                             &s_decompose_candidate_solution,
                                             &candidate_end_player);
                if((status == SOKOBAN_STATUS_OK) && (s_decompose_candidate_solution.solved != 0u))
                {
                    if((s_decompose_candidate_solution.move_count < best_move_count) ||
                       ((s_decompose_candidate_solution.move_count == best_move_count) &&
                        (s_decompose_candidate_solution.push_count < best_push_count)))
                    {
                        s_decompose_best_solution = s_decompose_candidate_solution;
                        best_end_player = candidate_end_player;
                        best_box_idx = box_idx;
                        best_goal_idx = goal_idx;
                        best_move_count = s_decompose_candidate_solution.move_count;
                        best_push_count = s_decompose_candidate_solution.push_count;
                    }
                }
            }
        }

        if((best_box_idx == SK_INVALID_CELL) || (best_goal_idx == SK_INVALID_CELL))
        {
            return SOKOBAN_STATUS_UNSOLVABLE;
        }

        if(!sk_append_solution(solution, &s_decompose_best_solution))
        {
            return SOKOBAN_STATUS_PATH_OVERFLOW;
        }

        remaining_boxes[best_box_idx] = 0u;
        remaining_goals[best_goal_idx] = 0u;
        current_player = best_end_player;
        remaining_count--;
    }

    solution->solved = 1u;
    solution->expanded_reverse = 0u;
    return SOKOBAN_STATUS_OK;
}

/* Public entry: pure walking BFS with boxes and walls as obstacles. */
sokoban_status_t sokoban_plan_walk(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                   uint8_t start_row,
                                   uint8_t start_col,
                                   uint8_t target_row,
                                   uint8_t target_col,
                                   sokoban_solution_t *solution)
{
    uint8_t start_cell;
    uint8_t target_cell;

    if((map == 0) || (solution == 0))
    {
        return SOKOBAN_STATUS_INVALID_ARGUMENT;
    }

    sk_init_lookup();
    if(!sk_cell_from_row_col(start_row, start_col, &start_cell) ||
       !sk_cell_from_row_col(target_row, target_col, &target_cell))
    {
        sk_solution_reset(solution);
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    return sk_plan_walk_cells(map, start_cell, target_cell, SK_INVALID_CELL, solution);
}

sokoban_direction_t sokoban_direction_turn(sokoban_direction_t heading, int8_t quarter_turns)
{
    int16_t value = (int16_t)heading + quarter_turns;
    while(value < 0)
    {
        value += 4;
    }
    return (sokoban_direction_t)(value & 3);
}

sokoban_body_direction_t sokoban_world_to_body(sokoban_direction_t heading,
                                               sokoban_direction_t world_direction)
{
    return (sokoban_body_direction_t)(((uint8_t)world_direction - (uint8_t)heading) & 3u);
}

static sokoban_direction_t sk_public_direction(uint8_t internal_dir)
{
    static const sokoban_direction_t directions[4] =
    {
        SOKOBAN_DIR_UP, SOKOBAN_DIR_DOWN, SOKOBAN_DIR_LEFT, SOKOBAN_DIR_RIGHT
    };
    return directions[internal_dir & 3u];
}

static int8_t sk_turn_delta(sokoban_direction_t from, sokoban_direction_t to)
{
    uint8_t delta = (uint8_t)(((uint8_t)to - (uint8_t)from) & 3u);
    return (delta == 3u) ? -1 : (int8_t)delta;
}

sokoban_status_t sokoban_plan_inspection(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                         sokoban_direction_t initial_heading,
                                         sokoban_inspection_plan_t *plan)
{
    uint8_t object_cells[SOKOBAN_MAX_OBJECTS];
    uint8_t object_types[SOKOBAN_MAX_OBJECTS];
    uint8_t object_indices[SOKOBAN_MAX_OBJECTS];
    uint8_t visited[SOKOBAN_MAX_OBJECTS];
    uint8_t object_count = 0u;
    uint8_t box_count = 0u;
    uint8_t bomb_count = 0u;
    uint8_t goal_count = 0u;
    uint8_t player_count = 0u;
    uint8_t current_cell = SK_INVALID_CELL;
    sokoban_direction_t heading = initial_heading;
    uint8_t row;
    uint8_t col;

    if((map == 0) || (plan == 0) || ((uint8_t)initial_heading > (uint8_t)SOKOBAN_DIR_LEFT))
    {
        return SOKOBAN_STATUS_INVALID_ARGUMENT;
    }

    (void)memset(plan, 0, sizeof(*plan));
    sk_solution_reset(&plan->route);
    (void)memset(visited, 0, sizeof(visited));
    sk_init_lookup();

    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            char ch = map[row][col];
            uint8_t cell = (uint8_t)(row * SOKOBAN_MAP_WIDTH + col);
            if(ch == '$')
            {
                if((box_count >= SOKOBAN_MAX_BOXES) || (object_count >= SOKOBAN_MAX_OBJECTS))
                {
                    return SOKOBAN_STATUS_TOO_MANY_BOXES;
                }
                object_cells[object_count] = cell;
                object_types[object_count] = (uint8_t)SOKOBAN_OBJECT_BOX;
                object_indices[object_count++] = box_count++;
            }
            else if(ch == '*')
            {
                if((box_count >= SOKOBAN_MAX_BOXES) || (bomb_count >= SOKOBAN_MAX_BOMBS))
                {
                    return SOKOBAN_STATUS_INVALID_MAP;
                }
                box_count++;
                bomb_count++;
            }
            else if((ch == '.') || (ch == 'G') || (ch == '+'))
            {
                if((goal_count >= SOKOBAN_MAX_BOXES) || (object_count >= SOKOBAN_MAX_OBJECTS))
                {
                    return SOKOBAN_STATUS_TOO_MANY_BOXES;
                }
                object_cells[object_count] = cell;
                object_types[object_count] = (uint8_t)SOKOBAN_OBJECT_GOAL;
                object_indices[object_count++] = goal_count++;
            }

            if((ch == '@') || (ch == '+'))
            {
                current_cell = cell;
                player_count++;
            }
            else if((ch != '#') && (ch != '-') && (ch != '.') && (ch != 'G') &&
                    (ch != '$') && (ch != '*'))
            {
                return SOKOBAN_STATUS_INVALID_MAP;
            }
        }
    }

    if((player_count != 1u) || (goal_count == 0u) ||
       ((uint8_t)(box_count - bomb_count) != goal_count))
    {
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    while(plan->event_count < object_count)
    {
        uint8_t best_object = SK_INVALID_CELL;
        uint8_t best_stand = SK_INVALID_CELL;
        sokoban_direction_t best_face = SOKOBAN_DIR_UP;
        int8_t best_turn = 0;
        uint16_t best_length = 0xFFFFu;
        uint8_t object_idx;

        for(object_idx = 0u; object_idx < object_count; object_idx++)
        {
            uint8_t side;
            if(visited[object_idx] != 0u)
            {
                continue;
            }
            for(side = 0u; side < 4u; side++)
            {
                int16_t stand_i = s_neighbor[object_cells[object_idx]][side];
                sokoban_direction_t face;
                int8_t turn;
                uint8_t turn_cost;
                uint8_t best_turn_cost;
                sokoban_status_t status;

                if(stand_i < 0)
                {
                    continue;
                }
                status = sk_plan_walk_cells(map, current_cell, (uint8_t)stand_i,
                                            SK_INVALID_CELL, &s_inspection_candidate_solution);
                if(status != SOKOBAN_STATUS_OK)
                {
                    continue;
                }
                face = sk_public_direction(sk_opposite_dir(side));
                turn = sk_turn_delta(heading, face);
                turn_cost = (uint8_t)((turn < 0) ? -turn : turn);
                best_turn_cost = (uint8_t)((best_turn < 0) ? -best_turn : best_turn);

                if((s_inspection_candidate_solution.move_count < best_length) ||
                   ((s_inspection_candidate_solution.move_count == best_length) &&
                    ((best_object == SK_INVALID_CELL) || (turn_cost < best_turn_cost) ||
                     ((turn_cost == best_turn_cost) &&
                      ((object_idx < best_object) ||
                       ((object_idx == best_object) && ((uint8_t)face < (uint8_t)best_face)))))))
                {
                    best_object = object_idx;
                    best_stand = (uint8_t)stand_i;
                    best_face = face;
                    best_turn = turn;
                    best_length = s_inspection_candidate_solution.move_count;
                    s_inspection_best_solution = s_inspection_candidate_solution;
                }
            }
        }

        if(best_object == SK_INVALID_CELL)
        {
            return SOKOBAN_STATUS_INSPECTION_UNREACHABLE;
        }
        if(!sk_append_solution(&plan->route, &s_inspection_best_solution))
        {
            return SOKOBAN_STATUS_PATH_OVERFLOW;
        }

        {
            sokoban_inspection_event_t *event = &plan->events[plan->event_count++];
            event->move_index = plan->route.move_count;
            event->object_index = object_indices[best_object];
            event->object_type = object_types[best_object];
            event->object_row = (uint8_t)(object_cells[best_object] / SOKOBAN_MAP_WIDTH);
            event->object_col = (uint8_t)(object_cells[best_object] % SOKOBAN_MAP_WIDTH);
            event->stand_row = (uint8_t)(best_stand / SOKOBAN_MAP_WIDTH);
            event->stand_col = (uint8_t)(best_stand % SOKOBAN_MAP_WIDTH);
            event->face_direction = best_face;
            event->quarter_turns = best_turn;
        }

        visited[best_object] = 1u;
        current_cell = best_stand;
        heading = best_face;
    }

    plan->route.solved = 1u;
    plan->final_row = (uint8_t)(current_cell / SOKOBAN_MAP_WIDTH);
    plan->final_col = (uint8_t)(current_cell % SOKOBAN_MAP_WIDTH);
    plan->final_heading = heading;
    return SOKOBAN_STATUS_OK;
}

/* Public entry: choose the internal wall whose removal gives the shortest walk. */
sokoban_status_t sokoban_find_best_bomb_wall(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                             uint8_t start_row,
                                             uint8_t start_col,
                                             uint8_t target_row,
                                             uint8_t target_col,
                                             sokoban_bomb_plan_t *plan)
{
    uint8_t start_cell;
    uint8_t target_cell;
    uint8_t row;
    uint8_t col;
    uint16_t best_len = 0xFFFFu;
    sokoban_status_t status;

    if((map == 0) || (plan == 0))
    {
        return SOKOBAN_STATUS_INVALID_ARGUMENT;
    }

    (void)memset(plan, 0, sizeof(*plan));
    sk_init_lookup();

    if(!sk_cell_from_row_col(start_row, start_col, &start_cell) ||
       !sk_cell_from_row_col(target_row, target_col, &target_cell))
    {
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    status = sk_plan_walk_cells(map, start_cell, target_cell, SK_INVALID_CELL, &s_bomb_walk_solution);
    if((status == SOKOBAN_STATUS_OK) && (s_bomb_walk_solution.solved != 0u))
    {
        best_len = s_bomb_walk_solution.move_count;
    }

    for(row = 1u; row + 1u < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 1u; col + 1u < SOKOBAN_MAP_WIDTH; col++)
        {
            uint8_t opened_wall;

            if(map[row][col] != '#')
            {
                continue;
            }

            opened_wall = (uint8_t)(row * SOKOBAN_MAP_WIDTH + col);
            status = sk_plan_walk_cells(map, start_cell, target_cell, opened_wall, &s_bomb_walk_solution);
            if((status == SOKOBAN_STATUS_OK) &&
               (s_bomb_walk_solution.solved != 0u) &&
               (s_bomb_walk_solution.move_count < best_len))
            {
                best_len = s_bomb_walk_solution.move_count;
                plan->found = 1u;
                plan->wall_row = row;
                plan->wall_col = col;
                plan->path_len_after_blast = s_bomb_walk_solution.move_count;
            }
        }
    }

    return (plan->found != 0u) ? SOKOBAN_STATUS_OK : SOKOBAN_STATUS_UNSOLVABLE;
}

/* Public entry: parse map -> initialize -> legacy single-direction A* -> rebuild path. */
sokoban_status_t sokoban_solve_bidirectional_astar(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                                   sokoban_solution_t *solution)
{
    sk_search_t forward_search;
    sokoban_status_t solver_status;
    uint16_t best_cost = 0xFFFFu;
    uint16_t best_forward_idx = SK_INVALID_NODE;

    if((map == 0) || (solution == 0))
    {
        return SOKOBAN_STATUS_INVALID_ARGUMENT;
    }

    sk_solution_reset(solution);
    sk_init_lookup();

    solver_status = sk_parse_map(map);
    if(solver_status != SOKOBAN_STATUS_OK)
    {
        return solver_status;
    }

    if(sk_is_goal_state(s_problem.start_boxes))
    {
        solution->solved = 1u;
        return SOKOBAN_STATUS_OK;
    }

    solver_status = sk_init_search(&forward_search);
    if(solver_status != SOKOBAN_STATUS_OK)
    {
        return solver_status;
    }

    while(forward_search.heap_size > 0u)
    {
        if((best_cost != 0xFFFFu) && (sk_heap_min_f(&forward_search) >= best_cost))
        {
            break;
        }

        solver_status = sk_expand_forward(&forward_search, &best_cost, &best_forward_idx);
        if(solver_status != SOKOBAN_STATUS_OK)
        {
            return solver_status;
        }
    }

    solution->expanded_forward = forward_search.expanded;
    solution->expanded_reverse = 0u;

    if(best_forward_idx == SK_INVALID_NODE)
    {
        return SOKOBAN_STATUS_UNSOLVABLE;
    }

    return sk_reconstruct(&forward_search, best_forward_idx, solution);
}

static uint8_t sk_labeled_wall(uint8_t cell, const uint8_t blast_centers[SOKOBAN_MAX_BOMBS])
{
    uint8_t row;
    uint8_t col;
    uint8_t bomb_slot;

    if(cell == SK_INVALID_CELL)
    {
        return 1u;
    }
    if(!sk_test_bit(s_labeled_problem.wall, cell))
    {
        return 0u;
    }
    row = (uint8_t)(cell / SOKOBAN_MAP_WIDTH);
    col = (uint8_t)(cell % SOKOBAN_MAP_WIDTH);
    if((row == 0u) || (col == 0u) || (row + 1u == SOKOBAN_MAP_HEIGHT) ||
       (col + 1u == SOKOBAN_MAP_WIDTH))
    {
        return 1u;
    }
    for(bomb_slot = 0u; bomb_slot < SOKOBAN_MAX_BOMBS; bomb_slot++)
    {
        uint8_t blast_center = blast_centers[bomb_slot];
        uint8_t blast_row;
        uint8_t blast_col;
        uint8_t dr;
        uint8_t dc;
        if(blast_center == SK_INVALID_CELL) continue;
        blast_row = (uint8_t)(blast_center / SOKOBAN_MAP_WIDTH);
        blast_col = (uint8_t)(blast_center % SOKOBAN_MAP_WIDTH);
        dr = (uint8_t)(row > blast_row ? row - blast_row : blast_row - row);
        dc = (uint8_t)(col > blast_col ? col - blast_col : blast_col - col);
        if((dr <= 1u) && (dc <= 1u)) return 0u;
    }
    return 1u;
}

static int8_t sk_labeled_bomb_slot(uint8_t box_index)
{
    uint8_t slot;
    for(slot = 0u; slot < s_labeled_problem.bomb_count; slot++)
    {
        if(s_labeled_problem.bomb_indices[slot] == box_index) return (int8_t)slot;
    }
    return -1;
}

static int8_t sk_labeled_box_at(const uint8_t box_cells[SOKOBAN_MAX_BOXES],
                                uint16_t active_boxes,
                                uint8_t cell)
{
    uint8_t idx;
    for(idx = 0u; idx < s_labeled_problem.box_count; idx++)
    {
        if(((active_boxes & ((uint16_t)1u << idx)) != 0u) && (box_cells[idx] == cell))
        {
            return (int8_t)idx;
        }
    }
    return -1;
}

static uint8_t sk_labeled_free(const uint8_t box_cells[SOKOBAN_MAX_BOXES],
                               uint16_t active_boxes,
                               const uint8_t blast_centers[SOKOBAN_MAX_BOMBS],
                               uint8_t cell)
{
    return (uint8_t)((cell != SK_INVALID_CELL) && !sk_labeled_wall(cell, blast_centers) &&
                     (sk_labeled_box_at(box_cells, active_boxes, cell) < 0));
}

static uint8_t sk_labeled_collect_reachable(const uint8_t box_cells[SOKOBAN_MAX_BOXES],
                                            uint16_t active_boxes,
                                            const uint8_t blast_centers[SOKOBAN_MAX_BOMBS],
                                            uint8_t start,
                                            uint8_t *out_cells,
                                            uint8_t *out_count,
                                            uint8_t *out_min)
{
    uint8_t head = 0u;
    uint8_t tail = 0u;
    uint8_t min_cell = start;

    if(!sk_labeled_free(box_cells, active_boxes, blast_centers, start))
    {
        return 0u;
    }
    (void)memset(s_bfs_visited, 0, sizeof(s_bfs_visited));
    s_bfs_queue[tail++] = start;
    s_bfs_visited[start] = 1u;
    *out_count = 0u;
    while(head < tail)
    {
        uint8_t cell = s_bfs_queue[head++];
        uint8_t dir;
        if(cell < min_cell)
        {
            min_cell = cell;
        }
        if(out_cells != 0)
        {
            out_cells[*out_count] = cell;
        }
        (*out_count)++;
        for(dir = 0u; dir < 4u; dir++)
        {
            int16_t next = s_neighbor[cell][dir];
            if((next >= 0) && (s_bfs_visited[(uint8_t)next] == 0u) &&
               sk_labeled_free(box_cells, active_boxes, blast_centers, (uint8_t)next))
            {
                s_bfs_visited[(uint8_t)next] = 1u;
                s_bfs_queue[tail++] = (uint8_t)next;
            }
        }
    }
    if(out_min != 0)
    {
        *out_min = min_cell;
    }
    return 1u;
}

static uint8_t sk_labeled_append_walk(const uint8_t box_cells[SOKOBAN_MAX_BOXES],
                                      uint16_t active_boxes,
                                      const uint8_t blast_centers[SOKOBAN_MAX_BOMBS],
                                      uint8_t start,
                                      uint8_t target,
                                      char *moves,
                                      uint16_t *io_count)
{
    uint8_t head = 0u;
    uint8_t tail = 0u;
    uint8_t cell;
    char reverse[SK_CELL_COUNT];
    uint16_t reverse_len = 0u;
    uint16_t count = *io_count;

    if(start == target)
    {
        return 1u;
    }
    (void)memset(s_bfs_visited, 0, sizeof(s_bfs_visited));
    (void)memset(s_bfs_parent, SK_INVALID_CELL, sizeof(s_bfs_parent));
    s_bfs_queue[tail++] = start;
    s_bfs_visited[start] = 1u;
    while(head < tail)
    {
        uint8_t dir;
        cell = s_bfs_queue[head++];
        for(dir = 0u; dir < 4u; dir++)
        {
            int16_t next = s_neighbor[cell][dir];
            if((next >= 0) && (s_bfs_visited[(uint8_t)next] == 0u) &&
               sk_labeled_free(box_cells, active_boxes, blast_centers, (uint8_t)next))
            {
                s_bfs_visited[(uint8_t)next] = 1u;
                s_bfs_parent[(uint8_t)next] = cell;
                s_bfs_parent_dir[(uint8_t)next] = dir;
                s_bfs_queue[tail++] = (uint8_t)next;
            }
        }
    }
    if(s_bfs_visited[target] == 0u)
    {
        return 0u;
    }
    cell = target;
    while(cell != start)
    {
        reverse[reverse_len++] = s_move_char[s_bfs_parent_dir[cell]];
        cell = s_bfs_parent[cell];
    }
    if((uint32_t)count + reverse_len > SOKOBAN_MAX_MOVES)
    {
        return 0u;
    }
    while(reverse_len > 0u)
    {
        moves[count++] = reverse[--reverse_len];
    }
    *io_count = count;
    return 1u;
}

static uint16_t sk_labeled_heuristic(const sk_labeled_node_t *node)
{
    uint16_t result = 0u;
    uint8_t idx;
    for(idx = 0u; idx < s_labeled_problem.box_count; idx++)
    {
        if(((node->active_boxes & ((uint16_t)1u << idx)) != 0u) &&
           ((s_labeled_problem.normal_box_mask & ((uint16_t)1u << idx)) != 0u))
        {
            result = (uint16_t)(result + sk_manhattan(node->box_cells[idx],
                                                       s_labeled_problem.box_goal_cells[idx]));
        }
    }
    return result;
}

static uint32_t sk_labeled_hash_node(const sk_labeled_node_t *node)
{
    uint32_t hash = 2166136261u;
    uint8_t idx;
    hash = (hash ^ node->active_boxes) * 16777619u;
    hash = (hash ^ node->remaining_goals) * 16777619u;
    hash = (hash ^ node->player) * 16777619u;
    for(idx = 0u; idx < SOKOBAN_MAX_BOMBS; idx++)
    {
        hash = (hash ^ node->blast_centers[idx]) * 16777619u;
    }
    for(idx = 0u; idx < s_labeled_problem.box_count; idx++)
    {
        if((node->active_boxes & ((uint16_t)1u << idx)) != 0u)
        {
            hash = (hash ^ ((uint32_t)idx << 8) ^ node->box_cells[idx]) * 16777619u;
        }
    }
    return hash % SK_LABELED_HASH_SIZE;
}

static uint8_t sk_labeled_same_state(const sk_labeled_node_t *lhs,
                                     const sk_labeled_node_t *rhs)
{
    uint8_t idx;
    if((lhs->active_boxes != rhs->active_boxes) ||
       (lhs->remaining_goals != rhs->remaining_goals) ||
       (lhs->player != rhs->player))
    {
        return 0u;
    }
    for(idx = 0u; idx < SOKOBAN_MAX_BOMBS; idx++)
    {
        if(lhs->blast_centers[idx] != rhs->blast_centers[idx]) return 0u;
    }
    for(idx = 0u; idx < s_labeled_problem.box_count; idx++)
    {
        if(((lhs->active_boxes & ((uint16_t)1u << idx)) != 0u) &&
           (lhs->box_cells[idx] != rhs->box_cells[idx]))
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t sk_labeled_heap_less(const sk_labeled_search_t *search, uint16_t lhs, uint16_t rhs)
{
    const sk_labeled_node_t *a = &search->nodes[lhs];
    const sk_labeled_node_t *b = &search->nodes[rhs];
    uint16_t af = (uint16_t)(a->g + a->h);
    uint16_t bf = (uint16_t)(b->g + b->h);
    if(af != bf) return (uint8_t)(af < bf);
    if(a->h != b->h) return (uint8_t)(a->h < b->h);
    return (uint8_t)(a->g > b->g);
}

static void sk_labeled_heap_swap(sk_labeled_search_t *search, uint16_t a, uint16_t b)
{
    uint16_t tmp = search->heap[a];
    search->heap[a] = search->heap[b];
    search->heap[b] = tmp;
    search->nodes[search->heap[a]].heap_pos = a;
    search->nodes[search->heap[b]].heap_pos = b;
}

static void sk_labeled_heap_up(sk_labeled_search_t *search, uint16_t pos)
{
    while(pos > 0u)
    {
        uint16_t parent = (uint16_t)((pos - 1u) >> 1);
        if(!sk_labeled_heap_less(search, search->heap[pos], search->heap[parent])) break;
        sk_labeled_heap_swap(search, pos, parent);
        pos = parent;
    }
}

static void sk_labeled_heap_down(sk_labeled_search_t *search, uint16_t pos)
{
    while(1)
    {
        uint16_t left = (uint16_t)(pos * 2u + 1u);
        uint16_t right = (uint16_t)(left + 1u);
        uint16_t best = pos;
        if((left < search->heap_size) && sk_labeled_heap_less(search, search->heap[left], search->heap[best])) best = left;
        if((right < search->heap_size) && sk_labeled_heap_less(search, search->heap[right], search->heap[best])) best = right;
        if(best == pos) break;
        sk_labeled_heap_swap(search, pos, best);
        pos = best;
    }
}

static void sk_labeled_heap_push(sk_labeled_search_t *search, uint16_t idx)
{
    search->heap[search->heap_size] = idx;
    search->nodes[idx].heap_pos = search->heap_size++;
    sk_labeled_heap_up(search, (uint16_t)(search->heap_size - 1u));
}

static uint16_t sk_labeled_heap_pop(sk_labeled_search_t *search)
{
    uint16_t result = search->heap[0];
    search->nodes[result].heap_pos = SK_INVALID_NODE;
    search->heap_size--;
    if(search->heap_size > 0u)
    {
        search->heap[0] = search->heap[search->heap_size];
        search->nodes[search->heap[0]].heap_pos = 0u;
        sk_labeled_heap_down(search, 0u);
    }
    return result;
}

static void sk_labeled_search_reset(sk_labeled_search_t *search)
{
    uint16_t idx;
    search->nodes = s_search_buffers.labeled.nodes;
    search->heap = s_search_buffers.labeled.heap;
    search->hash = s_search_buffers.labeled.hash;
    search->node_count = 0u;
    search->heap_size = 0u;
    search->expanded = 0u;
    for(idx = 0u; idx < SK_LABELED_HASH_SIZE; idx++) search->hash[idx] = SK_INVALID_NODE;
}

static uint16_t sk_labeled_find_state(const sk_labeled_search_t *search,
                                      const sk_labeled_node_t *candidate)
{
    uint16_t idx = search->hash[sk_labeled_hash_node(candidate)];
    while(idx != SK_INVALID_NODE)
    {
        if(sk_labeled_same_state(&search->nodes[idx], candidate)) return idx;
        idx = search->nodes[idx].hash_next;
    }
    return SK_INVALID_NODE;
}

static uint8_t sk_labeled_add(sk_labeled_search_t *search,
                              const sk_labeled_node_t *candidate,
                              uint16_t *out_idx)
{
    uint16_t idx = sk_labeled_find_state(search, candidate);
    if(idx != SK_INVALID_NODE)
    {
        if(candidate->g < search->nodes[idx].g)
        {
            uint16_t hash_next = search->nodes[idx].hash_next;
            uint16_t heap_pos = search->nodes[idx].heap_pos;
            search->nodes[idx] = *candidate;
            search->nodes[idx].hash_next = hash_next;
            search->nodes[idx].heap_pos = heap_pos;
            search->nodes[idx].flags &= (uint8_t)~SK_FLAG_CLOSED;
            if(heap_pos == SK_INVALID_NODE) sk_labeled_heap_push(search, idx);
            else sk_labeled_heap_up(search, heap_pos);
        }
        *out_idx = idx;
        return 1u;
    }
    if(search->node_count >= SK_LABELED_NODE_CAPACITY) return 0u;
    idx = search->node_count++;
    search->nodes[idx] = *candidate;
    search->nodes[idx].heap_pos = SK_INVALID_NODE;
    search->nodes[idx].hash_next = search->hash[sk_labeled_hash_node(candidate)];
    search->hash[sk_labeled_hash_node(candidate)] = idx;
    sk_labeled_heap_push(search, idx);
    *out_idx = idx;
    return 1u;
}

static uint8_t sk_complete_one_missing_label(sokoban_label_table_t *labels,
                                             uint8_t box_count,
                                             uint8_t goal_count)
{
    uint8_t missing_count = 0u;
    uint8_t missing_is_box = 0u;
    uint8_t missing_index = 0u;
    uint8_t missing_label = 0u;
    uint8_t idx;

    for(idx = 0u; idx < box_count; idx++)
    {
        if(sk_labeled_bomb_slot(idx) >= 0) continue;
        if(labels->box_labels[idx] == 0u)
        {
            missing_count++;
            missing_is_box = 1u;
            missing_index = idx;
        }
        else
        {
            missing_label ^= labels->box_labels[idx];
        }
    }
    for(idx = 0u; idx < goal_count; idx++)
    {
        if(labels->goal_labels[idx] == 0u)
        {
            missing_count++;
            missing_is_box = 0u;
            missing_index = idx;
        }
        else
        {
            missing_label ^= labels->goal_labels[idx];
        }
    }

    if(missing_count == 0u) return 1u;
    if((missing_count != 1u) || (missing_label == 0u) ||
       (missing_label == SOKOBAN_BOMB_LABEL)) return 0u;

    /* Paired labels cancel out, leaving the one unpaired label. */
    if(missing_is_box != 0u)
    {
        labels->box_labels[missing_index] = missing_label;
    }
    else
    {
        labels->goal_labels[missing_index] = missing_label;
    }
    return 1u;
}

static sokoban_status_t sk_parse_labeled_problem(
    const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
    uint8_t start_row,
    uint8_t start_col,
    const sokoban_label_table_t *labels)
{
    uint8_t row;
    uint8_t col;
    uint8_t box_count = 0u;
    uint8_t goal_count = 0u;
    uint8_t idx;
    sokoban_label_table_t resolved_labels;

    (void)memset(&s_labeled_problem, 0, sizeof(s_labeled_problem));
    for(idx = 0u; idx < SOKOBAN_MAX_BOMBS; idx++)
        s_labeled_problem.bomb_indices[idx] = SK_INVALID_CELL;
    for(idx = 0u; idx < SOKOBAN_MAX_BOXES; idx++) s_labeled_problem.box_goal_cells[idx] = SK_INVALID_CELL;
    for(row = 0u; row < SOKOBAN_MAP_HEIGHT; row++)
    {
        for(col = 0u; col < SOKOBAN_MAP_WIDTH; col++)
        {
            uint8_t cell = (uint8_t)(row * SOKOBAN_MAP_WIDTH + col);
            char ch = map[row][col];
            if(ch == '#') sk_set_bit(s_labeled_problem.wall, cell);
            else if(ch == '$')
            {
                if(box_count >= SOKOBAN_MAX_BOXES) return SOKOBAN_STATUS_TOO_MANY_BOXES;
                s_labeled_problem.box_cells[box_count++] = cell;
            }
            else if(ch == '*')
            {
                if(box_count >= SOKOBAN_MAX_BOXES) return SOKOBAN_STATUS_TOO_MANY_BOXES;
                if(s_labeled_problem.bomb_count >= SOKOBAN_MAX_BOMBS) return SOKOBAN_STATUS_INVALID_MAP;
                s_labeled_problem.bomb_indices[s_labeled_problem.bomb_count++] = box_count;
                s_labeled_problem.box_cells[box_count++] = cell;
            }
            else if((ch == '.') || (ch == 'G') || (ch == '+'))
            {
                if(goal_count >= SOKOBAN_MAX_BOXES) return SOKOBAN_STATUS_TOO_MANY_BOXES;
                s_labeled_problem.goal_cells[goal_count++] = cell;
            }
            else if((ch != '-') && (ch != '@')) return SOKOBAN_STATUS_INVALID_MAP;
        }
    }
    if((start_row >= SOKOBAN_MAP_HEIGHT) || (start_col >= SOKOBAN_MAP_WIDTH) ||
       (labels->box_count != box_count) || (labels->goal_count != goal_count) ||
       (goal_count == 0u) ||
       ((uint8_t)(box_count - s_labeled_problem.bomb_count) != goal_count))
    {
        return SOKOBAN_STATUS_INVALID_LABELS;
    }
    resolved_labels = *labels;
    if(!sk_complete_one_missing_label(&resolved_labels, box_count, goal_count))
    {
        return SOKOBAN_STATUS_INVALID_LABELS;
    }
    s_labeled_problem.box_count = box_count;
    s_labeled_problem.goal_count = goal_count;
    s_labeled_problem.start_player = (uint8_t)(start_row * SOKOBAN_MAP_WIDTH + start_col);
    if(sk_test_bit(s_labeled_problem.wall, s_labeled_problem.start_player) ||
       (sk_labeled_box_at(s_labeled_problem.box_cells, (uint16_t)((1u << box_count) - 1u),
                          s_labeled_problem.start_player) >= 0))
    {
        return SOKOBAN_STATUS_INVALID_MAP;
    }

    s_labeled_problem.all_goal_mask = (uint16_t)((1u << goal_count) - 1u);
    for(idx = 0u; idx < goal_count; idx++)
    {
        uint8_t other;
        if((resolved_labels.goal_labels[idx] == 0u) ||
           (resolved_labels.goal_labels[idx] == SOKOBAN_BOMB_LABEL))
            return SOKOBAN_STATUS_INVALID_LABELS;
        for(other = 0u; other < idx; other++)
            if(resolved_labels.goal_labels[other] ==
               resolved_labels.goal_labels[idx]) return SOKOBAN_STATUS_INVALID_LABELS;
    }
    for(idx = 0u; idx < box_count; idx++)
    {
        int8_t bomb_slot = sk_labeled_bomb_slot(idx);
        uint8_t other;
        uint8_t goal_idx;
        uint8_t found = 0u;
        uint8_t label = resolved_labels.box_labels[idx];
        if(bomb_slot >= 0)
        {
            if(label != SOKOBAN_BOMB_LABEL) return SOKOBAN_STATUS_INVALID_LABELS;
            continue;
        }
        if(label == 0u) return SOKOBAN_STATUS_INVALID_LABELS;
        if(label == SOKOBAN_BOMB_LABEL) return SOKOBAN_STATUS_INVALID_LABELS;
        for(other = 0u; other < idx; other++)
            if((sk_labeled_bomb_slot(other) < 0) &&
               (resolved_labels.box_labels[other] == label))
                return SOKOBAN_STATUS_INVALID_LABELS;
        for(goal_idx = 0u; goal_idx < goal_count; goal_idx++)
        {
            if(resolved_labels.goal_labels[goal_idx] == label)
            {
                s_labeled_problem.box_goal_cells[idx] = s_labeled_problem.goal_cells[goal_idx];
                s_labeled_problem.normal_box_mask |= (uint16_t)1u << idx;
                found = 1u;
                break;
            }
        }
        if(found == 0u) return SOKOBAN_STATUS_INVALID_LABELS;
    }
    return SOKOBAN_STATUS_OK;
}

static sokoban_status_t sk_labeled_reconstruct(const sk_labeled_search_t *search,
                                               uint16_t goal_idx,
                                               sokoban_solution_t *solution)
{
    uint16_t chain_len = 0u;
    uint16_t idx = goal_idx;
    uint8_t box_cells[SOKOBAN_MAX_BOXES];
    uint16_t active = (uint16_t)((1u << s_labeled_problem.box_count) - 1u);
    uint8_t blast_centers[SOKOBAN_MAX_BOMBS];
    uint8_t player = s_labeled_problem.start_player;
    uint16_t move_count = 0u;
    uint16_t push_count = 0u;
    uint8_t bomb_slot;

    sk_solution_reset(solution);
    for(bomb_slot = 0u; bomb_slot < SOKOBAN_MAX_BOMBS; bomb_slot++)
        blast_centers[bomb_slot] = SK_INVALID_CELL;
    while(idx != SK_INVALID_NODE)
    {
        if(chain_len >= SK_LABELED_NODE_CAPACITY) return SOKOBAN_STATUS_PATH_OVERFLOW;
        s_search_buffers.labeled.heap[chain_len++] = idx;
        idx = search->nodes[idx].parent;
    }
    (void)memcpy(box_cells, s_labeled_problem.box_cells, sizeof(box_cells));
    while(chain_len > 1u)
    {
        const sk_labeled_node_t *child = &search->nodes[s_search_buffers.labeled.heap[chain_len - 2u]];
        uint8_t box_from;
        if(!sk_labeled_append_walk(box_cells, active, blast_centers, player, child->action_from,
                                   solution->move_seq, &move_count))
            return SOKOBAN_STATUS_PATH_OVERFLOW;
        if(move_count >= SOKOBAN_MAX_MOVES) return SOKOBAN_STATUS_PATH_OVERFLOW;
        if(child->action_blast != 0u)
        {
            uint8_t slot = (uint8_t)(child->action_blast - 1u);
            uint8_t blast_center;
            if((slot >= SOKOBAN_MAX_BOMBS) || (solution->blast_count >= SOKOBAN_MAX_BOMBS))
                return SOKOBAN_STATUS_INVALID_MAP;
            blast_center = child->blast_centers[slot];
            solution->blast_move_indices[solution->blast_count] = move_count;
            solution->blast_rows[solution->blast_count] = (uint8_t)(blast_center / SOKOBAN_MAP_WIDTH);
            solution->blast_cols[solution->blast_count] = (uint8_t)(blast_center % SOKOBAN_MAP_WIDTH);
            solution->blast_count++;
        }
        solution->move_seq[move_count++] = s_move_char[child->action_dir];
        push_count++;
        box_from = (uint8_t)s_neighbor[child->action_from][child->action_dir];
        player = box_from;
        (void)memcpy(box_cells, child->box_cells, sizeof(box_cells));
        active = child->active_boxes;
        (void)memcpy(blast_centers, child->blast_centers, sizeof(blast_centers));
        chain_len--;
    }
    solution->move_seq[move_count] = '\0';
    solution->move_count = move_count;
    solution->push_count = push_count;
    solution->solved = 1u;
    return SOKOBAN_STATUS_OK;
}

static sokoban_status_t sk_labeled_astar(sokoban_solution_t *solution)
{
    sk_labeled_search_t search;
    sk_labeled_node_t start;
    uint16_t start_idx;
    uint16_t best_idx = SK_INVALID_NODE;
    uint16_t best_cost = 0xFFFFu;
    uint8_t reachable_count;
    uint8_t bomb_slot;

    sk_labeled_search_reset(&search);
    (void)memset(&start, 0, sizeof(start));
    (void)memcpy(start.box_cells, s_labeled_problem.box_cells, sizeof(start.box_cells));
    start.active_boxes = (uint16_t)((1u << s_labeled_problem.box_count) - 1u);
    start.remaining_goals = s_labeled_problem.all_goal_mask;
    start.parent = SK_INVALID_NODE;
    start.heap_pos = SK_INVALID_NODE;
    for(bomb_slot = 0u; bomb_slot < SOKOBAN_MAX_BOMBS; bomb_slot++)
        start.blast_centers[bomb_slot] = SK_INVALID_CELL;
    start.action_from = SK_INVALID_CELL;
    start.action_dir = SK_INVALID_CELL;
    if(!sk_labeled_collect_reachable(start.box_cells, start.active_boxes, start.blast_centers,
                                     s_labeled_problem.start_player, 0, &reachable_count, &start.player))
        return SOKOBAN_STATUS_INVALID_MAP;
    start.h = sk_labeled_heuristic(&start);
    if(!sk_labeled_add(&search, &start, &start_idx)) return SOKOBAN_STATUS_NO_MEMORY;

    while(search.heap_size > 0u)
    {
        uint16_t current_idx = sk_labeled_heap_pop(&search);
        sk_labeled_node_t *current = &search.nodes[current_idx];
        uint8_t reachable[SK_CELL_COUNT];
        uint8_t reachable_count_local = 0u;
        uint8_t reachable_idx;
        uint16_t min_f = (uint16_t)(current->g + current->h);
        if((best_cost != 0xFFFFu) && (min_f >= best_cost)) break;
        if((current->flags & SK_FLAG_CLOSED) != 0u) continue;
        current->flags |= SK_FLAG_CLOSED;
        search.expanded++;
        if(current->remaining_goals == 0u)
        {
            best_idx = current_idx;
            best_cost = current->g;
            continue;
        }
        if(!sk_labeled_collect_reachable(current->box_cells, current->active_boxes,
                                         current->blast_centers, current->player,
                                         reachable, &reachable_count_local, 0)) continue;
        for(reachable_idx = 0u; reachable_idx < reachable_count_local; reachable_idx++)
        {
            uint8_t dir;
            uint8_t player_cell = reachable[reachable_idx];
            for(dir = 0u; dir < 4u; dir++)
            {
                int16_t box_cell_i = s_neighbor[player_cell][dir];
                int8_t box_idx;
                int16_t target_i;
                sk_labeled_node_t child;
                uint8_t child_reachable_count;
                uint16_t child_idx;
                if(box_cell_i < 0) continue;
                box_idx = sk_labeled_box_at(current->box_cells, current->active_boxes, (uint8_t)box_cell_i);
                if(box_idx < 0) continue;
                target_i = s_neighbor[(uint8_t)box_cell_i][dir];
                if(target_i < 0) continue;
                child = *current;
                child.parent = current_idx;
                child.g = (uint16_t)(current->g + 1u);
                child.action_from = player_cell;
                child.action_dir = dir;
                child.action_blast = 0u;
                child.flags = 0u;
                child.heap_pos = SK_INVALID_NODE;
                child.hash_next = SK_INVALID_NODE;

                if(sk_labeled_wall((uint8_t)target_i, current->blast_centers))
                {
                    int8_t slot = sk_labeled_bomb_slot((uint8_t)box_idx);
                    uint8_t target_row = (uint8_t)((uint8_t)target_i / SOKOBAN_MAP_WIDTH);
                    uint8_t target_col = (uint8_t)((uint8_t)target_i % SOKOBAN_MAP_WIDTH);
                    if((slot < 0) || (current->blast_centers[(uint8_t)slot] != SK_INVALID_CELL) ||
                       (target_row == 0u) || (target_col == 0u) ||
                       (target_row + 1u == SOKOBAN_MAP_HEIGHT) || (target_col + 1u == SOKOBAN_MAP_WIDTH))
                        continue;
                    child.active_boxes &= (uint16_t)~((uint16_t)1u << (uint8_t)box_idx);
                    child.box_cells[(uint8_t)box_idx] = SK_INVALID_CELL;
                    child.blast_centers[(uint8_t)slot] = (uint8_t)target_i;
                    child.action_blast = (uint8_t)((uint8_t)slot + 1u);
                }
                else
                {
                    uint8_t goal_idx;
                    if(sk_labeled_box_at(current->box_cells, current->active_boxes, (uint8_t)target_i) >= 0) continue;
                    child.box_cells[(uint8_t)box_idx] = (uint8_t)target_i;
                    if(((s_labeled_problem.normal_box_mask & ((uint16_t)1u << (uint8_t)box_idx)) != 0u) &&
                       (s_labeled_problem.box_goal_cells[(uint8_t)box_idx] == (uint8_t)target_i))
                    {
                        child.active_boxes &= (uint16_t)~((uint16_t)1u << (uint8_t)box_idx);
                        child.box_cells[(uint8_t)box_idx] = SK_INVALID_CELL;
                        for(goal_idx = 0u; goal_idx < s_labeled_problem.goal_count; goal_idx++)
                            if(s_labeled_problem.goal_cells[goal_idx] == (uint8_t)target_i)
                                child.remaining_goals &= (uint16_t)~((uint16_t)1u << goal_idx);
                    }
                }
                if(!sk_labeled_collect_reachable(child.box_cells, child.active_boxes, child.blast_centers,
                                                 (uint8_t)box_cell_i, 0,
                                                 &child_reachable_count, &child.player)) continue;
                child.h = sk_labeled_heuristic(&child);
                if(!sk_labeled_add(&search, &child, &child_idx)) return SOKOBAN_STATUS_NO_MEMORY;
                if((child.remaining_goals == 0u) && (child.g < best_cost))
                {
                    best_idx = child_idx;
                    best_cost = child.g;
                }
            }
        }
    }
    if(best_idx == SK_INVALID_NODE) return SOKOBAN_STATUS_UNSOLVABLE;
    {
        sokoban_status_t status = sk_labeled_reconstruct(&search, best_idx, solution);
        solution->expanded_forward = search.expanded;
        return status;
    }
}

sokoban_status_t sokoban_solve_labeled(const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                                       uint8_t start_row,
                                       uint8_t start_col,
                                       const sokoban_label_table_t *labels,
                                       sokoban_solution_t *solution)
{
    sokoban_status_t status;
    if((map == 0) || (labels == 0) || (solution == 0)) return SOKOBAN_STATUS_INVALID_ARGUMENT;
    sk_solution_reset(solution);
    sk_init_lookup();
    status = sk_parse_labeled_problem(map, start_row, start_col, labels);
    if(status != SOKOBAN_STATUS_OK) return status;
    return sk_labeled_astar(solution);
}

/* Convert one U/D/L/R step into Chinese text for display. */
const char *sokoban_move_to_chinese(char move)
{
    switch(move)
    {
        case 'U': return "\xE4\xB8\x8A";
        case 'D': return "\xE4\xB8\x8B";
        case 'L': return "\xE5\xB7\xA6";
        case 'R': return "\xE5\x8F\xB3";
        default:  return "";
    }
}

/* Convert a whole U/D/L/R string into Chinese direction words. */
uint16_t sokoban_moves_to_chinese(const char *moves, uint16_t move_count, char *out, uint16_t out_capacity)
{
    uint16_t in_idx;
    uint16_t out_idx = 0u;

    if((moves == 0) || (out == 0) || (out_capacity == 0u))
    {
        return 0u;
    }

    for(in_idx = 0u; in_idx < move_count; in_idx++)
    {
        const char *word = sokoban_move_to_chinese(moves[in_idx]);
        while(*word != '\0')
        {
            if((uint16_t)(out_idx + 1u) >= out_capacity)
            {
                out[out_idx] = '\0';
                return out_idx;
            }
            out[out_idx++] = *word++;
        }
    }

    out[out_idx] = '\0';
    return out_idx;
}

/* Convert solver status enum into readable text. */
const char *sokoban_status_string(sokoban_status_t status)
{
    switch(status)
    {
        case SOKOBAN_STATUS_OK:                 return "OK";
        case SOKOBAN_STATUS_INVALID_ARGUMENT:   return "INVALID_ARGUMENT";
        case SOKOBAN_STATUS_INVALID_MAP:        return "INVALID_MAP";
        case SOKOBAN_STATUS_BOX_GOAL_MISMATCH:  return "BOX_GOAL_MISMATCH";
        case SOKOBAN_STATUS_TOO_MANY_BOXES:     return "TOO_MANY_BOXES";
        case SOKOBAN_STATUS_NO_MEMORY:          return "NO_MEMORY";
        case SOKOBAN_STATUS_PATH_OVERFLOW:      return "PATH_OVERFLOW";
        case SOKOBAN_STATUS_UNSOLVABLE:         return "UNSOLVABLE";
        case SOKOBAN_STATUS_INVALID_LABELS:     return "INVALID_LABELS";
        case SOKOBAN_STATUS_INSPECTION_UNREACHABLE: return "INSPECTION_UNREACHABLE";
        default:                                return "UNKNOWN";
    }
}
