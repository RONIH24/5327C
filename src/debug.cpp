#include "roboto/debug.hpp"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_chart.h"
#include "main.h"
#include <cstddef>
#include <string>

lv_obj_t *create_chart() {
  lv_obj_t *chart = lv_chart_create(lv_scr_act(), NULL);
  lv_obj_set_size(chart, 200, 150);
  lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);

  return chart;
}

lv_chart_series_t *create_series(lv_obj_t *chart, lv_color_t color) {
  lv_chart_series_t *chart_series = lv_chart_add_series(chart, color);
  return chart_series;
}

void update(void *pain) {
  Quack *wot = (Quack *)pain;
  pros::Motor vroom = *(wot->motor1);
  pros::Motor foom = *(wot->motor2);
  while (true) {
    double motor_rpm_1 = vroom.get_actual_velocity();
    double motor_rpm_2 = foom.get_actual_velocity();
    lv_chart_set_next(wot->chart, wot->series1, motor_rpm_1);
    lv_chart_set_next(wot->chart, wot->series2, motor_rpm_2);
    pros::delay(20);
  }
}

// todo: add pros task
void setup_graph(pros::Motor motor1, pros::Motor motor2) {
  lv_obj_t *chart = create_chart();
  lv_chart_series_t *series1 = create_series(chart, LV_COLOR_BLUE);
  lv_chart_series_t *series2 = create_series(chart, LV_COLOR_RED);
  Quack wheat = Quack{chart, series1, series2, &motor1, &motor2};
  pros::Task update_graph(update, (void *)&wheat, "graph");
}
