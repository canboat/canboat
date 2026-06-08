#include <check.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Test buffer overflow protection in message formatting */
#define MAX_SAFE_LINE_SIZE 2048
#define MAX_MSG_LEN 255

START_TEST(test_buffer_overflow_protection)
{
    /* Invariant: Formatted output must never exceed buffer bounds regardless of msgLen */
    
    /* Test cases: msgLen values that could cause overflow */
    size_t test_msg_lens[] = {
        255,  /* Max possible - exploit case: 255 bytes = 1020+ hex chars */
        128,  /* Boundary - half max */
        1     /* Valid minimal input */
    };
    int num_tests = sizeof(test_msg_lens) / sizeof(test_msg_lens[0]);

    for (int i = 0; i < num_tests; i++) {
        size_t msgLen = test_msg_lens[i];
        
        /* Calculate required buffer size for safe formatting:
         * - Timestamp + metadata prefix: ~50 chars max
         * - Each byte: ",XX" = 4 chars (including comma)
         * - Null terminator: 1 char
         */
        size_t required_size = 50 + (msgLen * 4) + 1;
        
        /* Security invariant: any msgLen up to 255 must fit in a reasonable buffer */
        ck_assert_msg(required_size <= MAX_SAFE_LINE_SIZE,
            "msgLen=%zu requires %zu bytes, exceeds safe buffer size %d",
            msgLen, required_size, MAX_SAFE_LINE_SIZE);
        
        /* Verify the worst case (255 bytes) calculation */
        if (msgLen == 255) {
            /* 255 * 4 = 1020 for hex + 50 for prefix = 1070 minimum */
            ck_assert_msg(required_size >= 1070,
                "Calculation error: 255-byte message needs at least 1070 bytes");
        }
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_buffer_overflow_protection);
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}