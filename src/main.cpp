#include <Arduino.h>
#include "pins.hpp"
#include "kart.hpp"
#include "ld06.hpp"
#include "dma.hpp"
#include "logger.hpp"
#include "pure_pursuit.hpp"
#include "f1tenth_gap_follow.hpp"
#include "naive_gap_follow.hpp"
#include "math.h"

// Robot control
TinyKart *tinyKart;

// LiDAR
LD06 ld06{};

// Scan processor
ScanBuilder scan_builder{180, 360, ScanPoint{0,0}};

/// Starts/stops the kart
void estop()
{
    logger.printf("Toggle Pause\n");

    tinyKart->toggle_pause();
    digitalToggle(LED_YELLOW);
}

void setup()
{
    // LEDs
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_YELLOW, HIGH);

    // Setup blue user button on the board to stop the kart
    pinMode(USER_BTN, INPUT);
    attachInterrupt(digitalPinToInterrupt(USER_BTN), estop, FALLING);

    // Init PWM
    analogWriteResolution(PWM_BITS); // Range of 0-4096
    analogWriteFrequency(PWM_FREQ);

    // Prepare kart for motion
    ESC esc{THROTTLE_PIN, PWM_MAX_DUTY, PWM_FREQ};
    tinyKart = new TinyKart{STEERING_PIN, esc, 0.3, 4.5};

    // Init DMA and UART for LiDAR
    dmaSerialRx5.begin(230'400, [&](volatile LD06Buffer buffer)
                       {
        // On each packet received, copy over to driver.
        ld06.add_buffer(buffer, 47); });

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
}

void loop()
{
    noInterrupts();
    auto res = ld06.get_scan();
    interrupts();

    // Check if we have a scan frame
    if (res)
    {
        auto scan_res = *res;

        // Check if frame erred
        if (scan_res)
        {
            auto maybe_scan = scan_builder.add_frame(scan_res.scan);

            // Check if we have a 180 degree scan built
            if (maybe_scan)
            {
                auto scan = *maybe_scan;
                auto maybe_gap = gap_follow::find_gap_bubble(scan, 0.5, true);
                logger.printf("*****START SCAN******\n");
                bool canMove = true;
                logger.printf("points: %hu\n", scan.size());



                if (maybe_gap) {
                    auto gap = *maybe_gap;
                    auto x = (gap.x * 100);
                    auto y = (gap.y * 100);
                    auto steering_command = pure_pursuit::calculate_command_to_point(tinyKart, gap, 1.5);


                    logger.printf("%hu, %hu", x, y);
///fix
                    if (abs(x) > 10) {
                        tinyKart->set_steering(-steering_command.steering_angle);
                    } else {
                        tinyKart->set_steering(0);
                    }

                    if (gap.dist(ScanPoint{0, 0}) < 0.2) {
                        //panic
                        tinyKart->set_neutral();
                    } else {
                        auto speed = 0.19;
                        /*auto speed = mapfloat(y, 30, 130, 0.05, 0.3);
                        
                        if (speed < 0 || speed > 0.7) {
                            speed = 0;
                        }*/
                        tinyKart->set_forward(steering_command.throttle_percent);
                    }


                }else{
                        tinyKart->set_steering(0);
                        tinyKart->set_neutral();

                }
                /*for (auto &pt : scan)
                {
                    pt.x*=100;
                    pt.y*=100;

                    if ((pt.x > -10 && pt.x < 10) && (pt.y > 0 && pt.y < 140))
                    {
                        logger.printf("\nIn box");
                        canMove = false;
                        break;
                    }
                }

                if (canMove)
                {
                    tinyKart->set_steering(0);
                    tinyKart->set_forward(0.2);
                }
                else
                {
                    tinyKart->set_neutral();
                }*/
                logger.printf("*****END SCAN******\n\n");
            }
        }
        else
        {
            switch (scan_res.error)
            {
            case ScanResult::Error::CRCFail:
                logger.printf("CRC error!\n");
                break;

            case ScanResult::Error::HeaderByteWrong:
                logger.printf("Header byte wrong!\n");
                break;
            }
        }
    }
}
