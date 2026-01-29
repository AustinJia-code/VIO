/**
 * @file euroc_player.hpp
 * @brief player for euroc dataset files
 */

#pragma once

#include "defs.h"
#include <fstream>
#include <map>
#include <optional>

class EurocPlayer
{
private:
    std::map<ns_t, std::string> cam0_map;
    std::map<ns_t, std::string>::iterator cam_it;
    std::map<ns_t, Eigen::Vector3d> gt_map;
    std::vector<IMUData> imu_data_storage;

    size_t imu_idx;

    std::string root;
    // 1403636599913555456 idle start
    // 1403636623963555584 idle end
    // 1403636661163555584 rapid downward motion
    // 1403636718013555456 starting from rapid downward, this spot doesnt move
    size_t STARTING_TS = 1403636599913555456; 

public:
    EurocPlayer (std::string path) : root (path)
    {
        imu_idx = 0;

        std::ifstream file (root + "/cam0/data.csv");
        std::string line;
        std::getline (file, line); // header
        
        while (std::getline (file, line))
        {
            std::stringstream ss (line);
            std::string ts, fname;
            std::getline (ss, ts, ',');
            std::getline (ss, fname, ',');
            fname.erase (fname.find_last_not_of (" \r\n\t") + 1);

            size_t ullts = std::stoull (ts);
            if (ullts < STARTING_TS)
                continue;

            cam0_map[std::stoull (ts)] = root + "/cam0/data/" + fname;
        }

        cam_it = cam0_map.begin ();

        // Load IMU Data
        std::ifstream imu_file (root + "/imu0/data.csv");
        std::getline (imu_file, line);
        while (std::getline (imu_file, line))
        {
            std::stringstream ss (line);
            std::string val;
            std::vector<double> row;
            while (std::getline (ss, val, ','))
                row.push_back (std::stod (val));

            if (row.size () >= 7)
            {
                IMUData m;
                m.time_ns = static_cast<ns_t> (row[0]);
                if (m.time_ns < STARTING_TS)
                    continue;

                // EuroC format: ts, gyros(x,y,z), accels(x,y,z)
                m.gyro = Eigen::Vector3d (row[1], row[2], row[3]);
                m.accel = Eigen::Vector3d (row[4], row[5], row[6]);
                imu_data_storage.push_back (m);
            }
        }
    }

    std::optional<std::pair<CamData, CamData>> get_next_stereo ()
    {
        if (cam_it == cam0_map.end ())
            return std::nullopt;
        
        CamData l = {};
        CamData r = {};

        l.time_ns = cam_it->first;
        l.img = cv::imread (cam_it->second, cv::IMREAD_GRAYSCALE);
        
        std::string r_path = cam_it->second;
        size_t pos = r_path.find ("cam0");
        r_path.replace (pos, 4, "cam1");
        r.img = cv::imread (r_path, cv::IMREAD_GRAYSCALE);
        r.time_ns = l.time_ns;

        cam_it++;

        return {std::make_pair (std::move (l), std::move (r))};
    }

    void load_ground_truth (std::string path)
    {
        std::ifstream file (path + "/state_groundtruth_estimate0/data.csv");
        std::string line;
        std::getline (file, line);

        while (std::getline (file, line))
        {
            std::stringstream ss (line);
            std::string ts_str, val;
            std::vector<double> row;
            
            std::getline (ss, ts_str, ',');
            while (std::getline (ss, val, ','))
                row.push_back (std::stod (val));
            
            if (row.size () >= 3)
                gt_map[std::stoull(ts_str)] = Eigen::Vector3d (row[0], row[1], row[2]);
        }
    }

    Eigen::Vector3d get_gt (ns_t ts) const
    {
        // Find the closest timestamp in GT for this frame
        auto it = gt_map.lower_bound (ts);
        if (it == gt_map.end ())
            return Eigen::Vector3d::Zero ();
        
        return it->second;
    }

    Eigen::Vector3d get_gt_init () const
    {
        return gt_map.begin ()->second;
    }

    std::vector<IMUData> get_imu_until (ns_t ts)
    {
        std::vector<IMUData> batch;
        
        while (imu_idx < imu_data_storage.size () &&
               imu_data_storage[imu_idx].time_ns <= ts)
        {
            IMUData m = {};
            
            m.time_ns = imu_data_storage[imu_idx].time_ns;
            m.accel = imu_data_storage[imu_idx].accel;
            m.gyro = imu_data_storage[imu_idx].gyro; 
            
            batch.push_back (m);
            imu_idx++;
        }
        
        return batch;
    }
};