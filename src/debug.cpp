#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_chart.h"
#include "main.h"
#include "pros/apix.h"
#include <tuple>

std::tuple<lv_obj_t *, lv_chart_series_t *> create_chart() {
  lv_obj_t *chart;
  chart = lv_chart_create(lv_scr_act(), NULL);
  lv_obj_set_size(chart, 200, 150);
  lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);

  lv_chart_series_t *chart_series = lv_chart_add_series(chart, LV_COLOR_GREEN);
  return std::make_tuple(chart, chart_series);
}

void update(lv_obj_t *chart, lv_chart_series_t *chart_series,
            pros::Motor flywheel) {
  while (true) {
    double current_rpm = flywheel.get_actual_velocity();
    lv_chart_set_next(chart, chart_series, current_rpm);
    pros::delay(20);
  }
}

// todo: add pros task
