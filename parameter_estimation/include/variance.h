//
// Created by havso on 07/10/2024.
//


#ifndef VARIANCE_H
#define VARIANCE_H


double variance(rapidcsv::Document doc, int row_) {
    auto data = doc.GetColumn<double>(row_);
    double mean{};
    double mu_i{};
    for (int i = 0; i < data.size(); i++) {
        mu_i += data[i];
    }
    mean = 1/data.size() * mu_i;

    double sum{};
    double var{};
    for (int i = 0; i < data.size(); i++) {
        sum += (mu_i * data[i])(mu_i * data[i]);
    }
    var = 1/data.size() * sum;
    return var;
}









#endif //VARIANCE_H
