#include "MainPlotter.h"
#include "DisplayUtils.h"

MainPlotter::MainPlotter(TFT_eSPI& tft) : tft(tft) {}

void MainPlotter::draw(const DataStore& data) {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);
    float zero_diff_mapped = DisplayUtils::mapf(0, MIN_DIFF_TEMP, data.MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
    int zero_diff_y = PLOT_Y_START + PLOT_HEIGHT - (int)zero_diff_mapped;
    tft.drawFastHLine(PLOT_X_START, zero_diff_y, PLOT_WIDTH, PLOT_ZERO_COLOR);

    if (PLOT_WIDTH > 1) {
        float min_voltage = 1000.0;
        float max_voltage = -1000.0;
        bool first_valid_voltage = true;

        for (int i = 0; i < PLOT_WIDTH; i++) {
            if (!isnan(data.voltage_values[i])) {
                if (first_valid_voltage) {
                    min_voltage = data.voltage_values[i];
                    max_voltage = data.voltage_values[i];
                    first_valid_voltage = false;
                } else {
                    min_voltage = fmin(min_voltage, data.voltage_values[i]);
                    max_voltage = fmax(max_voltage, data.voltage_values[i]);
                }
            }
        }

        min_voltage = fmax(min_voltage, 1.15f);
        max_voltage = fmin(max_voltage, 3.0f);

        if (min_voltage == max_voltage) {
            if (first_valid_voltage) {
                min_voltage = 0.5f;
                max_voltage = 1.0f;
            } else {
                float offset = 0.1f;
                min_voltage -= offset;
                max_voltage += offset;
                min_voltage = fmax(min_voltage, 0.5f);
                max_voltage = fmin(max_voltage, 3.0f);
            }
        }

        uint16_t grid_color = DisplayUtils::darkerColor(GRAPH_COLOR_VOLTAGE, 0.25f);
        int grid_x = PLOT_X_START + PLOT_WIDTH / 8;
        float target_voltages[] = {min_voltage, 1.25f, 1.38f, 1.55f, max_voltage};
        int num_targets = sizeof(target_voltages) / sizeof(target_voltages[0]);

        tft.setTextColor(grid_color);
        tft.setTextSize(1);

        for (int i = 0; i < num_targets; i++) {
            float voltage = target_voltages[i];
            float mapped_y = DisplayUtils::mapf(voltage, min_voltage, max_voltage, 0, PLOT_HEIGHT);
            int grid_y = PLOT_Y_START + PLOT_HEIGHT - (int)mapped_y;
            tft.drawFastHLine(grid_x, grid_y, PLOT_WIDTH, grid_color);
            if (voltage != min_voltage && voltage != max_voltage) {
                tft.drawFloat(voltage, 2, grid_x - 5, grid_y + 8, 1);
            }
        }

        for (int i = 0; i < PLOT_WIDTH - 1; i++) {
            if (!isnan(data.voltage_values[i]) && !isnan(data.voltage_values[i + 1])) {
                int y_voltage_prev = PLOT_Y_START + PLOT_HEIGHT - (int)DisplayUtils::mapf(data.voltage_values[i], min_voltage, max_voltage, 0, PLOT_HEIGHT);
                int y_voltage_current = PLOT_Y_START + PLOT_HEIGHT - (int)DisplayUtils::mapf(data.voltage_values[i + 1], min_voltage, max_voltage, 0, PLOT_HEIGHT);
                tft.drawLine(PLOT_X_START + i, y_voltage_prev, PLOT_X_START + i + 1, y_voltage_current, GRAPH_COLOR_VOLTAGE);
            }
        }

        tft.setTextColor(GRAPH_COLOR_VOLTAGE);
        tft.setTextSize(1);
        tft.drawFloat(min_voltage, 2, PLOT_X_START + PLOT_WIDTH - 40, PLOT_Y_START + PLOT_HEIGHT - 15, 1);
        tft.drawFloat(max_voltage, 2, PLOT_X_START + PLOT_WIDTH - 40, PLOT_Y_START, 1);
    }

    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        if (!isnan(data.temp1_values[i]) && !isnan(data.temp1_values[i + 1])) {
            int y1_prev = PLOT_Y_START + PLOT_HEIGHT - (int)DisplayUtils::mapf(data.temp1_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            int y1_current = PLOT_Y_START + PLOT_HEIGHT - (int)DisplayUtils::mapf(data.temp1_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y1_prev, PLOT_X_START + i + 1, y1_current, GRAPH_COLOR_1);
        }
        if (!isnan(data.temp2_values[i]) && !isnan(data.temp2_values[i + 1])) {
            int y2_prev = PLOT_Y_START + PLOT_HEIGHT - (int)DisplayUtils::mapf(data.temp2_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            int y2_current = PLOT_Y_START + PLOT_HEIGHT - (int)DisplayUtils::mapf(data.temp2_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y2_prev, PLOT_X_START + i + 1, y2_current, GRAPH_COLOR_2);
        }
        if (!isnan(data.diff_values[i]) && !isnan(data.diff_values[i + 1])) {
            int y_diff_prev = PLOT_Y_START + PLOT_HEIGHT - (int)DisplayUtils::mapf(data.diff_values[i], MIN_DIFF_TEMP, data.MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
            int y_diff_current = PLOT_Y_START + PLOT_HEIGHT - (int)DisplayUtils::mapf(data.diff_values[i + 1], MIN_DIFF_TEMP, data.MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_diff_prev, PLOT_X_START + i + 1, y_diff_current, GRAPH_COLOR_DIFF);
        }
        if (!isnan(data.current_values[i]) && !isnan(data.current_values[i + 1])) {
            int y_current_prev = PLOT_Y_START + PLOT_HEIGHT - (int)DisplayUtils::mapf(data.current_values[i], MIN_CURRENT, MAX_CURRENT, 0, PLOT_HEIGHT);
            int y_current_current = PLOT_Y_START + PLOT_HEIGHT - (int)DisplayUtils::mapf(data.current_values[i + 1], MIN_CURRENT, MAX_CURRENT, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_current_prev, PLOT_X_START + i + 1, y_current_current, GRAPH_COLOR_CURRENT);
        }
    }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    int label_line_height = 8;

    tft.setCursor(PLOT_X_START, LABEL_Y_START);
    tft.print("T1: ");
    if (!isnan(data.temp1_values[PLOT_WIDTH - 1])) {
        tft.printf("%.2f C", data.temp1_values[PLOT_WIDTH - 1]);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_1, TFT_BLACK);
    tft.print(" R");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 100, LABEL_Y_START);
    tft.print("V: ");
    if (!isnan(data.voltage_values[PLOT_WIDTH - 1])) {
        tft.printf("%.3f V", data.voltage_values[PLOT_WIDTH - 1]);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_VOLTAGE, TFT_BLACK);
    tft.print(" Y");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 100, LABEL_Y_START+label_line_height * 1);
    tft.print("I: ");
    if (!isnan(data.current_values[PLOT_WIDTH - 1])) {
        tft.printf("%.3f A", data.current_values[PLOT_WIDTH - 1]);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_CURRENT, TFT_BLACK);
    tft.print(" M");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 0);
    tft.print("VCC:");
    // if (!isnan(sensors.getVCC())) {
    //     tft.printf("%.2f mV", sensors.getVCC());
    // } else {
    //     tft.print("Error");
    // }

    tft.setCursor(PLOT_X_START, LABEL_Y_START + label_line_height);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("T2: ");
    if (!isnan(data.temp2_values[PLOT_WIDTH - 1])) {
        tft.printf("%.2f C", data.temp2_values[PLOT_WIDTH - 1]);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_2, TFT_BLACK);
    tft.print(" G");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 1);
    tft.print("mV :");
    // if (!isnan(sensors.getRawMillivolts1())) {
    //     tft.printf("%.2f mV", sensors.getRawMillivolts1());
    // } else {
    //     tft.print("Error");
    // }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 2);
    tft.print("%  :");
    if (!isnan(data.dutyCycle)) {
        tft.printf("%u  ", data.dutyCycle);
    } else {
        tft.print("Error");
    }

    tft.setCursor(PLOT_X_START, LABEL_Y_START + 2 * label_line_height);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("dT: ");
    if (!isnan(data.diff_values[PLOT_WIDTH - 1])) {
        tft.printf("%.2f C", data.diff_values[PLOT_WIDTH - 1]);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_DIFF, TFT_BLACK);
    tft.print(" B");
}