#include "parameter_estimation/include/ParameterEstimation.h"
#include "parameter_estimation/include/fusion.hpp"
#include "parameter_estimation/include/variance.h"
#include "state_estimation/include/kalman_filter.h"
#include "parameter_estimation/include/fusionv2.hpp"
#include <iostream>
#include <vector>
#include <fstream>

void writeCSV(const std::string &filename, const std::vector<std::vector<std::string> > &data) {
    std::ofstream file(filename); // Create and open the file
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file " << filename << std::endl;
        return;
    }

    // Loop through the data and write it to the file
    for (const auto &row: data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i]; // Write each element
            if (i != row.size() - 1) // Add a comma except after the last element
                file << ",";
        }
        file << "\n"; // New line after each row
    }

    file.close(); // Don't forget to close the file
    std::cout << "Data successfully written to " << filename << std::endl;
}

std::vector<std::string> vectorXdToString(const Eigen::VectorXd &vec) {
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

    rapidcsv::Document documentAccel2("../dataset/2-vibrations_accel.csv");
    rapidcsv::Document documentOrientations2("../dataset/2-vibrations_orientations.csv");
    rapidcsv::Document documentWrench2("../dataset/2-vibrations_wrench.csv");

    rapidcsv::Document documentAccel3("../dataset/3-vibrations-contact_accel.csv");
    rapidcsv::Document documentOrientations3("../dataset/3-vibrations-contact_orientations.csv");
    rapidcsv::Document documentWrench3("../dataset/3-vibrations-contact_wrench.csv");

    ParameterEstimation parameter_estimation(biasDoc);

    auto forceBias = parameter_estimation.getForceBiasVector();
    auto torqueBias = parameter_estimation.getTorqueBiasVector();
    auto imuBias = parameter_estimation.getImuBiasVector();
    auto massEstimate = parameter_estimation.getMassEstimate();
    auto centerMassEstimate = parameter_estimation.getCenterMassVector();

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

    Fusion2 fusion2(massEstimate, centerMassEstimate, varForce, varTorque, varAccel, forceBias, torqueBias, imuBias);
    fusion2.insertData(documentAccel3, documentWrench3, documentOrientations3);

    MatrixXd P = MatrixXd::Identity(9, 9); // inital estimate, vetta faen egnt
    VectorXd X = VectorXd::Ones(9);

    kalman_filter kalman_filter(
        X,
        P,
        fusion2.getA(),
        fusion2.getB(),
        fusion2.getH(),
        fusion2.getQ(),
        fusion2.getR()
    );

    std::vector<VectorXd> stateVariableVectorForPlotting;
    std::vector<VectorXd> ftsEstimateVector;

    fusion2.updateMatrices(); // Initilize

    try {
        while (!fusion2.isFinished()) {
            fusion2.updateMatrices();

            kalman_filter.priori(fusion2.getU(), fusion2.getQ());
            kalman_filter.posteriori(fusion2.getZ(), fusion2.getR(), fusion2.getH());

            stateVariableVectorForPlotting.push_back(kalman_filter.getX());
            ftsEstimateVector.push_back(fusion2.getZc()*kalman_filter.getX());
        }
    } catch (const std::out_of_range& e) {
        std::cout << e.what() << std::endl;
    }

    std::vector<std::vector<std::string> > csvData;
    std::vector<std::vector<std::string> > csvDataEstimate;
    for (const auto &vec: stateVariableVectorForPlotting) {
        csvData.push_back(vectorXdToString(vec));
    }
    for (const auto &vec: ftsEstimateVector) {
        csvDataEstimate.push_back(vectorXdToString(vec));
    }

    writeCSV(csvPath, csvData);
    writeCSV(csvPathEstimate, csvDataEstimate);
}

