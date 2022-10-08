#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_chart.h"
#include "main.h"
#include "display/lv_core/lv_obj.h"
#include <cstddef>
#include <string>

typedef struct {
  lv_obj_t *chart;
  lv_chart_series_t *chart_series;
  pros::Motor *flywheel;
} Quack;

typedef struct {
  lv_obj_t *chart;
  lv_chart_series_t *chart_series;
} Charty;

Charty cruelty() {
lv_obj_t *chart = lv_chart_create(lv_scr_act(), NULL);
lv_obj_set_size(chart, 200, 150);
lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);

lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
lv_chart_series_t *chart_series = lv_chart_add_series(chart, LV_COLOR_GREEN);
return Charty{chart, chart_series};
} 

void update(void* pain) {
  Quack* wot = (Quack*)pain;
  auto vroom = *(wot->flywheel);
  while (true) {
    double current_rpm = vroom.get_actual_velocity();
    lv_chart_set_next(wot->chart, wot->chart_series, current_rpm);
    pros::delay(20);
  }
}

// todo: add pros task
void setup_graph(pros::Motor flywheel) {
  Charty fart = cruelty();
  Quack wheat = Quack{fart.chart, fart.chart_series, &flywheel};
  pros::Task update_graph(update,(void*) &wheat, "graph");
}
