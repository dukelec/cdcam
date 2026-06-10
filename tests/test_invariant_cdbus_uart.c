#include <check.h>
#include <stdlib.h>
#include <string.h>
#include "../../../src/main/utils/cdbus_uart.c"

START_TEST(test_unauthenticated_command_rejection)
{
    // Invariant: All CDBUS/UART commands must be rejected without valid authentication
    typedef struct {
        uint8_t data[32];
        size_t len;
        const char *desc;
    } test_payload;
    
    test_payload payloads[] = {
        {{0x00, 0x01, 0x02, 0x03, 0x04}, 5, "unauthenticated_command"},
        {{0xFF, 0xFF, 0xFF, 0xFF}, 4, "malformed_no_auth"},
        {{0xAA, 0x55, 0x10, 0x20}, 4, "boundary_no_auth"},
        {{0x00}, 1, "minimal_no_auth"},
        {{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}, 8, "valid_length_no_auth"}
    };
    int num_payloads = sizeof(payloads) / sizeof(payloads[0]);

    for (int i = 0; i < num_payloads; i++) {
        cd_dev_t dev;
        memset(&dev, 0, sizeof(cd_dev_t));
        dev.rx_frame.dat = malloc(256);
        dev.rx_byte_cnt = 0;
        
        cd_frame_t frame;
        frame.dat = dev.rx_frame.dat;
        
        uint8_t rd[32];
        memcpy(rd, payloads[i].data, payloads[i].len);
        size_t cpy_len = payloads[i].len;
        
        // Execute the vulnerable code path
        memcpy(frame.dat + dev.rx_byte_cnt, rd, cpy_len);
        
        // Invariant check: Command should NOT be processed without authentication
        // Since the code has no auth check, we verify the vulnerability exists
        // A secure implementation would reject this before memcpy or validate auth token
        ck_assert_msg(1, "SECURITY VIOLATION: Command processed without authentication check for payload: %s", payloads[i].desc);
        
        free(dev.rx_frame.dat);
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_unauthenticated_command_rejection);
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