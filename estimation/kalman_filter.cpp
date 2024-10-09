#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>
#include "include/kalman_filter.h"
#include "include/variance.h"


int main() {

    rapidcsv::Document doc0("../dataset/0-steady-state_accel.csv");
    rapidcsv::Document doc1("../dataset/0-steady-state_wrench.csv");

    double varAx = variance(doc0, 0);
    double varAy = variance(doc0, 1);
    double varAz = variance(doc0, 2);

    double varFx = variance(doc0, 2);
    double varFy = variance(doc0, 1);
    double varFz = variance(doc0, 2);
    double varTx = variance(doc0, 3);
    double varTy = variance(doc0, 4);
    double varTz = variance(doc0, 5);



    return 0;


}
