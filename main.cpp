#include "parameter_estimation/include/ParameterEstimation.h"
#include "parameter_estimation/include/fusion.hpp"
#include "parameter_estimation/include/variance.h"
#include "state_estimation/include/kalman_filter.h"
#include <iostream>
#include <vector>
#include <fstream>

void writeCSV(const std::string& filename, const std::vector<std::vector<std::string>>& data) {
    std::ofstream file(filename);  // Create and open the file
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file " << filename << std::endl;
        return;
    }

    // Loop through the data and write it to the file
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];  // Write each element
            if (i != row.size() - 1)  // Add a comma except after the last element
                file << ",";
        }
        file << "\n";  // New line after each row
    }

    file.close();  // Don't forget to close the file
    std::cout << "Data successfully written to " << filename << std::endl;
}

std::vector<std::string> vectorXdToString(const Eigen::VectorXd& vec) {
    std::vector<std::string> result(vec.size());
    for (int i = 0; i < vec.size(); ++i) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << vec[i]; // Adjust precision as needed
        result[i] = oss.str();
    }
    return result;
}

int main() {

    rapidcsv::Document biasDoc("../dataset/0-calibration_fts-accel.csv");
    rapidcsv::Document accelVarianceDoc("../dataset/0-steady-state_accel.csv");
    rapidcsv::Document wrenchVarianceDoc("../dataset/0-steady-state_wrench.csv");

    rapidcsv::Document documentAccel("../dataset/1-baseline_accel.csv");
    rapidcsv::Document documentOrientations("../dataset/1-baseline_orientations.csv");
    rapidcsv::Document documentWrench("../dataset/1-baseline_wrench.csv");

    ParameterEstimation parameter_estimation(biasDoc);

    auto forceBias = parameter_estimation.getForceBiasVector();
    auto torqueBias = parameter_estimation.getTorqueBiasVector();
    auto imuBias = parameter_estimation.getImuBiasVector();
    auto massEstimate = parameter_estimation.getMassEstimate();
    auto centerMassEstimate = parameter_estimation.getCenterMassVector();

    //std::cout << forceBias << " fb: " << torqueBias << " me: " << massEstimate << "cme: "<< centerMassEstimate << std::endl;

    std::vector<double> varForce(3);
    std::vector<double> varTorque(3);
    std::vector<double> varAccel(3);

    //NOE SKJER HÆR
    {
        varForce[0] = variance(wrenchVarianceDoc, 0);
        varForce[1] = variance(wrenchVarianceDoc, 1);
        varForce[2] = variance(wrenchVarianceDoc, 2);
        varTorque[0] = variance(wrenchVarianceDoc, 3);
        varTorque[1] = variance(wrenchVarianceDoc, 4);
        varTorque[2] = variance(wrenchVarianceDoc, 5);
        varAccel[0] = variance(accelVarianceDoc, 0);
        varAccel[1] = variance(accelVarianceDoc, 1);
        varAccel[2] = variance(accelVarianceDoc, 2);
    }

    Fusion fusion(massEstimate,centerMassEstimate,varForce,varTorque,varAccel,forceBias,torqueBias,imuBias);
    fusion.insertData(documentAccel,documentWrench,documentOrientations);
    fusion.updateMatrix(0);

    MatrixXd P = MatrixXd::Identity(9,9); // inital estimate, vetta faen egnt
    VectorXd X = VectorXd::Zero(9);

    kalman_filter kalman_filter(
        fusion.getX(),
        P,
        fusion.getA(),
        fusion.getB(),
        fusion.getH(),
        fusion.getQ(),
        fusion.getR()
        );

    std::vector<VectorXd> stateVariableVectorForPlotting;

    for (int i = 0; i < documentAccel.GetRowCount(); i++) {
        kalman_filter.priori(fusion.getU(), fusion.getQ());
        fusion.updateMatrix(i);
        kalman_filter.posteriori(fusion.getZ());
        stateVariableVectorForPlotting.push_back(kalman_filter.getX());
    }

    std::vector<std::vector<std::string>> csvData;
    for (const auto& vec : stateVariableVectorForPlotting) {
        csvData.push_back(vectorXdToString(vec));
    }

    std::string filename = csvPath;
    writeCSV(filename,csvData);

    /*k

    for (int i = 0; i < documentAccel.GetRowCount(); i++) {
        kalman_filter().predict();
        kalman_filter().update(z);
    }
    */
}

