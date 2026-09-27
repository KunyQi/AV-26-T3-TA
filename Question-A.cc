// Part A: This is an extension task that requires you to decode sensor data from CAN log files.
// CAN (Controller Area Network) is a communication standard used in automotive applications (including Redback cars)
// to allow communication between sensors and controllers.
//
// Your Task: Using the signal definitions in SteeringBench.dbc, read each CAN capture in data/
// and turn it into a CSV with one row per decoded frame:
// t,u_commanded,y_measured
// eg:
// 0,15.0,0.0
// 0.005,15.0,0.0
// ...
// where t is the frame timestamp minus the first kept frame's timestamp (s), u_commanded is
// the decoded CmdAngularRate (deg/s), and y_measured is the decoded MeasuredAngle (deg).
// The above values are not real numbers; they are only there to show the expected data output format.
// Do this for all three captures:
// data/step_test.log       ->  data/step_test.csv
// data/reversal_test.log   ->  data/reversal_test.csv
// data/deadband_test.log   ->  data/deadband_test.csv
//
// The Row type, writeCsv(), and main() below are provided -- they loop the three logs, call your
// decodeLog(), and write the CSV in exactly the format above. You just need to implement decodeLog().
//
// You do not need to use any external libraries. Use the resources below to understand how to
// extract sensor data.
// Hint: Think about manual bit masking and shifting, data types required,
// what formats are used to represent values, etc.
// Resources:
// https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial
// https://www.csselectronics.com/pages/can-dbc-file-database-intro
//
// Sanity check: plot your CSVs (python3 plot_data.py) and compare against the pre-plotted
// data/*.png files -- they should match.
//
// Build & run (from the TA/ folder):
//     c++ -std=c++17 Question-A.cc -o decode
//     ./decode

#include <cstdio>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    std::vector<Row> rows;

    std::ifstream input(path);
    if (!input) {
        std::fprintf(stderr, "Could not open %s\n", path.c_str());
        return rows;
    }

    bool haveOrigin = false;
    double origin = 0.0;
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream fields(line);
        char openParen = 0, closeParen = 0;
        double timestamp = 0.0;
        std::string interfaceName, frame;
        if (!(fields >> openParen >> timestamp >> closeParen >> interfaceName >> frame) ||
            openParen != '(' || closeParen != ')') {
            continue;
        }

        const std::size_t separator = frame.find('#');
        if (separator == std::string::npos) continue;

        unsigned long frameId = 0;
        try {
            frameId = std::stoul(frame.substr(0, separator), nullptr, 16);
        } catch (const std::exception&) {
            continue;
        }
        if (frameId != 0x200) continue;  // STEER_ActuatorLog (DBC message 512)

        const std::string payload = frame.substr(separator + 1);
        if (payload.size() != 16) continue;  // this DBC message contains 8 bytes

        std::uint8_t bytes[8]{};
        bool validPayload = true;
        for (std::size_t i = 0; i < 8; ++i) {
            try {
                bytes[i] = static_cast<std::uint8_t>(std::stoul(payload.substr(i * 2, 2), nullptr, 16));
            } catch (const std::exception&) {
                validPayload = false;
                break;
            }
        }
        if (!validPayload) continue;

        // DBC signals are Intel (little-endian), signed 16-bit values.
        auto signed16 = [&bytes](std::size_t firstByte) -> int {
            int raw = static_cast<int>(bytes[firstByte]) |
                      (static_cast<int>(bytes[firstByte + 1]) << 8);
            if (raw >= 0x8000) raw -= 0x10000;
            return raw;
        };

        if (!haveOrigin) {
            origin = timestamp;
            haveOrigin = true;
        }
        rows.push_back({timestamp - origin,
                        signed16(2) * 0.1,
                        signed16(0) * 0.1});
    }

    return rows;
}

// Provided -- writes the rows to a CSV in the required format. Do not change.
void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "t,u_commanded,y_measured\n";
    for (const Row& r : rows)
        f << r.t << "," << r.u_commanded << "," << r.y_measured << "\n";
}

// Provided -- runs decodeLog() + writeCsv() for each of the three captures.
int main() {
    const char* names[] = {"step_test", "reversal_test", "deadband_test"};
    for (const char* n : names) {
        const std::string in  = std::string("data/") + n + ".log";
        const std::string out = std::string("data/") + n + ".csv";
        const std::vector<Row> rows = decodeLog(in);
        writeCsv(out, rows);
        std::printf("%-14s %6zu frames -> %s\n", n, rows.size(), out.c_str());
    }
    return 0;
}
