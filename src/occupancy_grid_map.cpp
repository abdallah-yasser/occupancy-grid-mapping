#include <bits/stdc++.h>
using namespace std;

pair<int, int> worldToGrid(double x, double y, double min_x, double min_y, double res) {
    int gx = (int)((x - min_x) / res);
    int gy = (int)((y - min_y) / res);
    return {gx, gy};
}

vector<pair<int, int>> BresenhamLine(int x0, int y0, int x1, int y1) {
    vector<pair<int, int>> cells;
    int dx = abs(x1 - x0), dy = -abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    while (true) {
        cells.push_back({x0, y0});
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    return cells;
}

int main(int argc, char* argv[]) {
    // Usage: occupancy_grid_map [input.csv] [output_dir]
    string input_path = (argc > 1) ? argv[1] : "data/robot.csv";
    string out_dir    = (argc > 2) ? argv[2] : "results";

    ifstream file(input_path);
    if (!file.is_open()) {
        cerr << "error: cannot open " << input_path << "\n";
        return 1;
    }

    string line;
    getline(file, line);
    vector<string> headers;
    stringstream ss(line);
    string col;
    while (getline(ss, col, ',')) {
        transform(col.begin(), col.end(), col.begin(), ::tolower);
        col.erase(remove_if(col.begin(), col.end(), ::isspace), col.end());
        headers.push_back(col);
    }

    int idx_x = find(headers.begin(), headers.end(), "pos_x") - headers.begin();
    int idx_y = find(headers.begin(), headers.end(), "pos_y") - headers.begin();
    int idx_head = find(headers.begin(), headers.end(), "heading") - headers.begin();
    vector<int> idx_ultra(4);
    for (int i = 0; i < 4; i++)
        idx_ultra[i] = find(headers.begin(), headers.end(), "ultrasound" + to_string(i)) - headers.begin();

    vector<array<double, 7>> data;
    while (getline(file, line)) {
        stringstream s2(line);
        string val;
        vector<double> row;
        while (getline(s2, val, ',')) row.push_back(stod(val));
        array<double, 7> entry;
        entry[0] = row[idx_x];
        entry[1] = row[idx_y];
        entry[2] = row[idx_head];
        for (int i = 0; i < 4; i++) entry[3 + i] = row[idx_ultra[i]] * 343.0 / 2.0;
        data.push_back(entry);
    }
    file.close();

    double res = 0.05;
    double pad = 1.0;
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (auto &r : data) {
        min_x = min(min_x, r[0]);
        max_x = max(max_x, r[0]);
        min_y = min(min_y, r[1]);
        max_y = max(max_y, r[1]);
    }
    min_x -= pad; max_x += pad;
    min_y -= pad; max_y += pad;

    int width = ceil((max_x - min_x) / res);
    int height = ceil((max_y - min_y) / res);
    vector<vector<double>> log_odds(height, vector<double>(width, 0.0));

    double half_len = 0.10, half_wid = 0.05;
    vector<array<double, 3>> sensors = {
        {+half_len, +half_wid,  M_PI/4},
        {+half_len, -half_wid,  3*M_PI/4},
        {-half_len, -half_wid, -3*M_PI/4},
        {-half_len, +half_wid, -M_PI/4}
    };

    double P_occ = 0.85, P_free = 0.35;
    double LO_occ = log(P_occ / (1 - P_occ));
    double LO_free = log(P_free / (1 - P_free));
    double max_range = 2.0;
    double cone_half = M_PI / 12;
    int rays_per_cone = 9;   

    for (auto &row : data) {
        double px = row[0], py = row[1], head = row[2];
        for (int s = 0; s < 4; s++) {
            double ox = sensors[s][0], oy = sensors[s][1], ang = sensors[s][2];
            double sx = px + cos(head) * ox - sin(head) * oy;
            double sy = py + sin(head) * ox + cos(head) * oy;
            auto [gx0, gy0] = worldToGrid(sx, sy, min_x, min_y, res);

            double beam_center = head + ang;
            double dist = min(row[3 + s], max_range);

            for (int k = 0; k < rays_per_cone; k++) {
                double da = -cone_half + k * (2 * cone_half / (rays_per_cone - 1));
                double beam_ang = beam_center + da;
                double ex = sx + cos(beam_ang) * dist;
                double ey = sy + sin(beam_ang) * dist;
                auto [gx1, gy1] = worldToGrid(ex, ey, min_x, min_y, res);

                auto ray = BresenhamLine(gx0, gy0, gx1, gy1);
                for (size_t i = 0; i + 1 < ray.size(); i++) {
                    int cx = ray[i].first, cy = ray[i].second;
                    if (cx >= 0 && cx < width && cy >= 0 && cy < height)
                        log_odds[cy][cx] += LO_free;
                }
                if (dist < max_range) {
                    int cx = ray.back().first, cy = ray.back().second;
                    if (cx >= 0 && cx < width && cy >= 0 && cy < height)
                        log_odds[cy][cx] += LO_occ;
                }
            }
        }
    }

    vector<vector<unsigned char>> image(height, vector<unsigned char>(width));
    vector<vector<double>> prob_map(height, vector<double>(width));
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double p = 1.0 - 1.0 / (1.0 + exp(log_odds[y][x]));
            prob_map[y][x] = p;
            image[y][x] = (unsigned char)((1.0 - p) * 255);
        }
    }

    ofstream pgm(out_dir + "/occupancy_grid.pgm", ios::binary);
    pgm << "P5\n" << width << " " << height << "\n255\n";
    for (int y = 0; y < height; y++) pgm.write((char*)image[y].data(), width);
    pgm.close();

    ofstream csv(out_dir + "/occupancy_grid.csv");
    csv << fixed << setprecision(3);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            csv << prob_map[y][x];
            if (x < width - 1) csv << ",";
        }
        csv << "\n";
    }
    csv.close();

    cout << "Occupancy grid saved to " << out_dir << "/occupancy_grid.pgm and " << out_dir << "/occupancy_grid.csv\n";
    return 0;
}


