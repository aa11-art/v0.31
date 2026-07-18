#include "test.h"
#include "isr.h"
#include "uart.h"
#include <string.h>

#define LABEL_TEST_OBJECT_TYPE       (1u)
#define LABEL_TEST_OBJECT_INDEX      (0u)
#define LABEL_TEST_OBJECT_ROW        (0u)
#define LABEL_TEST_OBJECT_COL        (0u)
#define LABEL_TEST_BOOT_WAIT_MS      (3000u)
#define LABEL_TEST_REPLY_TIMEOUT_MS  (2000u)
#define LABEL_TEST_POLL_MS           (10u)

static const char g_test_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#--------------#",
    "#--------------#",
    "#----.---.-----#",
    "#----$---$-----#",
    "#----@---------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static const char g_single_box_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#--------------#",
    "#--------------#",
    "#----@$.-------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static const char g_with_star_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#--------------#",
    "#------*-------#",
    "#----@$.-------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static const char g_unsolvable_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#$-------------#",
    "#@------------.#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static const char g_labeled_bomb_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#-------#------#",
    "#-------#------#",
    "#-------#------#",
    "#-----$-#-.----#",
    "#------*#------#",
    "#-----@-#------#",
    "#-------#------#",
    "#-------#------#",
    "#-------#------#",
    "#-------#------#",
    "################"
};

static const char g_two_bomb_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "################",
    "################",
    "################",
    "################",
    "#@*#*#$.-------#",
    "################",
    "################",
    "################",
    "################",
    "################",
    "################"
};

static const char g_three_bomb_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "################",
    "################",
    "################",
    "################",
    "#@*#*#*#$.-----#",
    "################",
    "################",
    "################",
    "################",
    "################",
    "################"
};

static const char g_optional_bombs_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#-*------------#",
    "#------*-------#",
    "#--------------#",
    "#----@$.-------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static const char g_too_many_bombs_map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE] =
{
    "################",
    "#****----------#",
    "#@$.-----------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};

static void run_direction_tests(void)
{
    uint8_t heading;
    uint8_t world;
    uint8_t failures = 0u;
    for(heading = 0u; heading < 4u; heading++)
    {
        for(world = 0u; world < 4u; world++)
        {
            uint8_t expected = (uint8_t)((world - heading) & 3u);
            if((uint8_t)sokoban_world_to_body((sokoban_direction_t)heading,
                                              (sokoban_direction_t)world) != expected)
            {
                failures++;
            }
        }
    }
    if((sokoban_direction_turn(SOKOBAN_DIR_UP, 1) != SOKOBAN_DIR_RIGHT) ||
       (sokoban_direction_turn(SOKOBAN_DIR_UP, -1) != SOKOBAN_DIR_LEFT) ||
       (sokoban_direction_turn(SOKOBAN_DIR_UP, 2) != SOKOBAN_DIR_DOWN))
    {
        failures++;
    }
    printf("[direction] failures=%u\r\n", failures);
}

static void run_labeled_bomb_test(void)
{
    sokoban_label_table_t labels;
    sokoban_solution_t solution;
    sokoban_status_t status;
    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 1u;
    labels.goal_count = 1u;
    labels.box_labels[0] = 7u;
    labels.goal_labels[0] = 7u;
    status = sokoban_solve_labeled(g_single_box_map, 3u, 5u, &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.solved == 0u) || (solution.blast_count != 0u))
    {
        printf("[labeled_no_bomb] unexpected result\r\n");
    }

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 2u;
    labels.goal_count = 1u;
    labels.box_labels[0] = 1u;
    labels.box_labels[1] = SOKOBAN_BOMB_LABEL;
    labels.goal_labels[0] = 1u;
    status = sokoban_solve_labeled(g_labeled_bomb_map, 6u, 6u, &labels, &solution);
    printf("[labeled_bomb] status=%s solved=%u blasts=%u blast=%u,%u move=%u\r\n",
           sokoban_status_string(status), solution.solved, solution.blast_count,
           solution.blast_rows[0], solution.blast_cols[0], solution.move_count);
    if((status != SOKOBAN_STATUS_OK) || (solution.solved == 0u) ||
       (solution.blast_count != 1u) ||
       (solution.blast_rows[0] != 5u) || (solution.blast_cols[0] != 8u))
    {
        printf("[labeled_bomb] unexpected result\r\n");
    }
}

static void run_multi_bomb_tests(void)
{
    sokoban_label_table_t labels;
    sokoban_solution_t solution;
    sokoban_inspection_plan_t plan;
    sokoban_status_t status;

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 3u;
    labels.goal_count = 1u;
    labels.box_labels[0] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[1] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[2] = 7u;
    labels.goal_labels[0] = 7u;
    status = sokoban_solve_labeled(g_two_bomb_map, 5u, 1u, &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.blast_count != 2u) ||
       (solution.blast_rows[0] != 5u) || (solution.blast_cols[0] != 3u) ||
       (solution.blast_rows[1] != 5u) || (solution.blast_cols[1] != 5u))
    {
        printf("[two_bombs] unexpected result\r\n");
    }

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 4u;
    labels.goal_count = 1u;
    labels.box_labels[0] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[1] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[2] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[3] = 9u;
    labels.goal_labels[0] = 9u;
    status = sokoban_solve_labeled(g_three_bomb_map, 5u, 1u, &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.blast_count != 3u) ||
       (solution.blast_cols[0] != 3u) || (solution.blast_cols[1] != 5u) ||
       (solution.blast_cols[2] != 7u))
    {
        printf("[three_bombs] unexpected result\r\n");
    }

    (void)memset(&labels, 0, sizeof(labels));
    labels.box_count = 3u;
    labels.goal_count = 1u;
    labels.box_labels[0] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[1] = SOKOBAN_BOMB_LABEL;
    labels.box_labels[2] = 4u;
    labels.goal_labels[0] = 4u;
    status = sokoban_solve_labeled(g_optional_bombs_map, 4u, 5u, &labels, &solution);
    if((status != SOKOBAN_STATUS_OK) || (solution.blast_count != 0u))
    {
        printf("[optional_bombs] unexpected result\r\n");
    }
    status = sokoban_plan_inspection(g_optional_bombs_map, SOKOBAN_DIR_UP, &plan);
    if((status != SOKOBAN_STATUS_OK) || (plan.event_count != 2u))
    {
        printf("[bomb_inspection] unexpected result\r\n");
    }
    status = sokoban_plan_inspection(g_too_many_bombs_map, SOKOBAN_DIR_UP, &plan);
    if(status != SOKOBAN_STATUS_INVALID_MAP)
    {
        printf("[too_many_bombs] unexpected result\r\n");
    }
}

static void run_inspection_test(void)
{
    sokoban_inspection_plan_t plan;
    sokoban_status_t status = sokoban_plan_inspection(g_test_map, SOKOBAN_DIR_UP, &plan);
    printf("[inspection] status=%s events=%u move=%u final=%u,%u heading=%u\r\n",
           sokoban_status_string(status), plan.event_count, plan.route.move_count,
           plan.final_row, plan.final_col, (uint8_t)plan.final_heading);
    if((status != SOKOBAN_STATUS_OK) || (plan.event_count != 4u) ||
       (plan.final_row >= SOKOBAN_MAP_HEIGHT) || (plan.final_col >= SOKOBAN_MAP_WIDTH))
    {
        printf("[inspection] unexpected result\r\n");
    }
}

static void run_case(const char *name,
                     const char map[SOKOBAN_MAP_HEIGHT][SOKOBAN_MAP_STRIDE],
                     uint8_t expect_ok)
{
    sokoban_solution_t solution;
    sokoban_status_t status;

    status = sokoban_solve_decomposed(map, &solution);
    printf("[%s] status=%s solved=%u move=%u push=%u exp_f=%u exp_r=%u\r\n",
           name,
           sokoban_status_string(status),
           solution.solved,
           solution.move_count,
           solution.push_count,
           solution.expanded_forward,
           solution.expanded_reverse);

    if(expect_ok)
    {
        if((status != SOKOBAN_STATUS_OK) || (solution.solved == 0u))
        {
            printf("[%s] unexpected failure\r\n", name);
        }
    }
    else
    {
        if(status == SOKOBAN_STATUS_OK)
        {
            printf("[%s] unexpected success\r\n", name);
        }
    }
}

void sokoban_debug_once(void)
{
    sokoban_solution_t solution;
    sokoban_status_t status;
    char cn_buf[128];

    run_case("single_box_goal", g_single_box_map, 1u);
    run_case("double_box_regression", g_test_map, 1u);
    run_case("bomb_rejected_by_plain_solver", g_with_star_map, 0u);
    run_case("unsolvable", g_unsolvable_map, 0u);
    run_direction_tests();
    run_inspection_test();
    run_labeled_bomb_test();
    run_multi_bomb_tests();

    status = sokoban_solve_decomposed(g_test_map, &solution);

    printf("status=%s\r\n", sokoban_status_string(status));

    if(status == SOKOBAN_STATUS_OK)
    {
        sokoban_moves_to_chinese(solution.move_seq,
                                 solution.move_count,
                                 cn_buf,
                                 sizeof(cn_buf));

        printf("solved=%u\r\n", solution.solved);
        printf("move_count=%u\r\n", solution.move_count);
        printf("push_count=%u\r\n", solution.push_count);
        printf("expanded_f=%u\r\n", solution.expanded_forward);
        printf("expanded_r=%u\r\n", solution.expanded_reverse);
        printf("path_udlr=%s\r\n", solution.move_seq);
        printf("path_cn=%s\r\n", cn_buf);
			ips200_show_uint(0,1,solution.solved,8);
			ips200_show_uint(0,16,solution.move_count,8);
			ips200_show_uint(0,32,solution.push_count,8);
			ips200_show_uint(0,48,solution.expanded_forward,8);
			ips200_show_uint(0,64,solution.expanded_reverse,8);
			ips200_show_string (0,80,solution.move_seq);

    }
}

void label_uart_request_test_once(void)
{
    uint8_t request[8];
    uint8_t rx_ready;
    uint8_t rx_type;
    uint8_t rx_index;
    uint8_t rx_label;
    uint8_t frame_seen;
    uint8_t tail_ok;
    uint8_t check_ok;
    uint8_t match_ok;
    uint8_t comm_ok;
    uint8_t recognition_ok;
    uint32_t waited_ms = 0u;

    request[0] = 0xA5u;
    request[1] = 0xB5u;
    request[2] = LABEL_TEST_OBJECT_TYPE;
    request[3] = LABEL_TEST_OBJECT_INDEX;
    request[4] = LABEL_TEST_OBJECT_ROW;
    request[5] = LABEL_TEST_OBJECT_COL;
    request[6] = (uint8_t)(
        request[2] ^ request[3] ^ request[4] ^ request[5]);
    request[7] = 0xC5u;

    ips200_clear();
    ips200_show_string(0, 0, "UART4 LABEL TEST");
    ips200_show_string(0, 16, "MODE:DIGIT ONLY");
    ips200_show_string(0, 32, "TX:A5 B5 01 00 00 00 01 C5");
    ips200_show_string(0, 48, "BOOT WAIT:3000MS");
    ips200_show_string(0, 64, "VEHICLE TASKS:OFF");
    system_delay_ms(LABEL_TEST_BOOT_WAIT_MS);

    __disable_irq();
    label_rx_ready = 0u;
    label_rx_type = 0u;
    label_rx_index = 0u;
    label_rx_value = 0u;
    label_rx_frame_seen = 0u;
    label_rx_tail_ok = 0u;
    label_rx_check_ok = 0u;
    __enable_irq();

    ips200_show_string(0, 48, "TX STATUS:SENDING");
    label_uart_write_request(request, sizeof(request));
    ips200_show_string(0, 48, "TX STATUS:SENT   ");
    ips200_show_string(0, 80, "RX STATUS:WAITING");

    while(waited_ms < LABEL_TEST_REPLY_TIMEOUT_MS)
    {
        if(label_rx_ready != 0u)
        {
            break;
        }
        system_delay_ms(LABEL_TEST_POLL_MS);
        waited_ms += LABEL_TEST_POLL_MS;
    }

    __disable_irq();
    rx_ready = label_rx_ready;
    rx_type = label_rx_type;
    rx_index = label_rx_index;
    rx_label = label_rx_value;
    frame_seen = label_rx_frame_seen;
    tail_ok = label_rx_tail_ok;
    check_ok = label_rx_check_ok;
    label_rx_ready = 0u;
    __enable_irq();

    match_ok = (uint8_t)(
        (rx_ready != 0u) &&
        (rx_type == LABEL_TEST_OBJECT_TYPE) &&
        (rx_index == LABEL_TEST_OBJECT_INDEX));
    comm_ok = (uint8_t)(
        (rx_ready != 0u) &&
        (tail_ok != 0u) &&
        (check_ok != 0u) &&
        (match_ok != 0u));
    recognition_ok = (uint8_t)(
        (comm_ok != 0u) && (rx_label != 0u));

    ips200_clear();
    ips200_show_string(0, 0, "UART4 LABEL TEST");
    ips200_show_string(0, 16, "TX:A5 B5 01 00 00 00 01 C5");
    ips200_show_string(0, 32, "TX STATUS:SENT");

    ips200_show_string(0, 48, "RX STATUS:");
    if(rx_ready != 0u)
    {
        ips200_show_string(88, 48, "VALID");
    }
    else if(frame_seen != 0u)
    {
        ips200_show_string(88, 48, "INVALID");
    }
    else
    {
        ips200_show_string(88, 48, "TIMEOUT");
    }

    ips200_show_string(0, 64, "RX TYPE:");
    ips200_show_string(0, 80, "RX INDEX:");
    ips200_show_string(0, 96, "RX LABEL:");
    if(rx_ready != 0u)
    {
        ips200_show_uint(88, 64, rx_type, 3);
        ips200_show_uint(88, 80, rx_index, 3);
        ips200_show_uint(88, 96, rx_label, 3);
    }
    else
    {
        ips200_show_string(88, 64, "N/A");
        ips200_show_string(88, 80, "N/A");
        ips200_show_string(88, 96, "N/A");
    }

    ips200_show_string(0, 112, "TAIL:");
    ips200_show_string(88, 112,
        (frame_seen == 0u) ? "N/A" :
        ((tail_ok != 0u) ? "PASS" : "FAIL"));
    ips200_show_string(0, 128, "XOR:");
    ips200_show_string(88, 128,
        (frame_seen == 0u) ? "N/A" :
        ((check_ok != 0u) ? "PASS" : "FAIL"));
    ips200_show_string(0, 144, "MATCH:");
    ips200_show_string(88, 144,
        (rx_ready == 0u) ? "N/A" :
        ((match_ok != 0u) ? "PASS" : "FAIL"));
    ips200_show_string(0, 160, "WAIT MS:");
    ips200_show_uint(88, 160, waited_ms, 4);
    ips200_show_string(0, 176, "COMM:");
    ips200_show_string(88, 176,
        (comm_ok != 0u) ? "PASS" : "FAIL");
    ips200_show_string(0, 192, "RECOG:");
    ips200_show_string(88, 192,
        (recognition_ok != 0u) ? "PASS" : "REJECT");
    ips200_show_string(0, 208, "OVERALL:");
    ips200_show_string(88, 208,
        (recognition_ok != 0u) ? "PASS" : "FAIL");
    ips200_show_string(0, 224, "NO RETRY; TASKS OFF");
}
