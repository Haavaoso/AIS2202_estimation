#include "estimation/kalman_filter.h"
#include "rapidcsv.h"

int main()

{
    rapidcsv::Document doc("../dataset/0-calibration_fts-accel.csv", rapidcsv::LabelParams(0, 0));
    return 0;
}
