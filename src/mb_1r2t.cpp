#include <mb_1r2t/mb_1r2t.hpp>

#include <geometry_msgs/msg/point32.hpp>

#include <algorithm>
#include <limits>

MB_1r2t::MB_1r2t()
    : rclcpp::Node("mb_1r2t_node")
{
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    m_port = get_parameter("port").as_string();

    declare_parameter<std::string>("frame_id", "lidar");
    m_frame_id = get_parameter("frame_id").as_string();

    m_timer = create_wall_timer(std::chrono::milliseconds(1),
        std::bind(&MB_1r2t::publish_loop, this));

    m_laser_scan_publisher = create_publisher<sensor_msgs::msg::LaserScan>("/laser_scan", 10);
    m_point_cloud_publisher = create_publisher<sensor_msgs::msg::PointCloud>("/point_cloud", 10);

    m_serial_device = std::make_unique<SerialDevice>(*this, m_port);

    m_laser_scan_msg.range_min = RANGE_MIN;
    m_laser_scan_msg.range_max = RANGE_MAX;
    m_laser_scan_msg.ranges.assign(SAMPLES_PER_SCAN, HUGE_VAL);
    m_laser_scan_msg.intensities.assign(SAMPLES_PER_SCAN, 0.0);

    m_laser_scan_msg.header.frame_id = m_frame_id;
    m_point_cloud_msg.header.frame_id = m_frame_id;

    m_last_scan_time = now();

    RCLCPP_INFO(get_logger(), "started");
}

void MB_1r2t::publish_loop()
{
    parse_packet();
}

void MB_1r2t::publish_laser_scan()
{
    auto [min_angle_scan, max_angle_scan] = std::minmax_element(m_scan_results.begin(), m_scan_results.end(), [](const ScanResult& a, const ScanResult& b) {
        return a.angle < b.angle;
    });
    float min_angle = min_angle_scan->angle, max_angle = max_angle_scan->angle;

    RCLCPP_DEBUG(get_logger(), "start: %f, end: %f, size: %d", min_angle, max_angle, m_scan_results.size());

    for (ScanResult& s : m_scan_results) {
        int i = (s.angle - min_angle) / (max_angle - min_angle) * (float)SAMPLES_PER_SCAN;
        if (s.distance < m_laser_scan_msg.ranges[i]) {
            // TODO: use interpolation instead of assigning the distance from a single point
            m_laser_scan_msg.ranges[i] = s.distance;
            m_laser_scan_msg.intensities[i] = s.intensity;
        }
    }

    rclcpp::Duration scan_time = now() - m_last_scan_time;

    m_laser_scan_msg.scan_time = scan_time.seconds();
    m_laser_scan_msg.time_increment = scan_time.seconds() / (float)(SAMPLES_PER_SCAN - 1);
    m_laser_scan_msg.angle_min = min_angle;
    m_laser_scan_msg.angle_max = max_angle;
    m_laser_scan_msg.angle_increment = (max_angle - min_angle) / (float)(SAMPLES_PER_SCAN - 1);

    m_laser_scan_publisher->publish(m_laser_scan_msg);

    m_scan_results.clear();
    m_laser_scan_msg.ranges.assign(SAMPLES_PER_SCAN, HUGE_VAL);
    m_laser_scan_msg.intensities.assign(SAMPLES_PER_SCAN, 0.0);
}

void MB_1r2t::publish_point_cloud()
{
    m_point_cloud_msg.header.stamp = now();

    m_point_cloud_publisher->publish(m_point_cloud_msg);

    m_point_cloud_msg.points.clear();
}

void MB_1r2t::scan_done()
{
    publish_point_cloud();
    publish_laser_scan();

    m_laser_scan_msg.header.stamp = now();
    m_last_scan_time = now();
}

void MB_1r2t::scan_data()
{
    int16_t diff = m_packet.stop_angle - m_packet.start_angle;
    if (m_packet.stop_angle < m_packet.start_angle) {
        diff = 0xB400 - m_packet.start_angle + m_packet.stop_angle;
    }

    int16_t angle_per_sample = 0;
    if (diff > 1) {
        angle_per_sample = diff / (m_packet.data_length - 1);
    }

    for (int i = 0; i < m_packet.data_length; ++i) {
        uint16_t index = i * 3;
        uint8_t intensity = m_packet.data[index + 0];
        uint8_t distance_L = m_packet.data[index + 1];
        uint8_t distance_H = m_packet.data[index + 2];

        uint16_t distance = (uint16_t)(distance_H << 8) + (uint16_t)distance_L;
        float distancef = (float)distance / 4000.0;

        float step = M_PI * 2;
        float angle = (m_packet.start_angle + angle_per_sample * i);
        float anglef = (step * (angle / 0xB400));
        float angle_inv = (M_PI * 2) - anglef;

        ScanResult result;
        result.angle = angle_inv;
        result.distance = distancef;
        result.intensity = intensity;

        m_scan_results.emplace_back(result);

        geometry_msgs::msg::Point32 point;
        point.x = cos(angle_inv) * distancef;
        point.y = sin(angle_inv) * distancef;
        point.z = 0;

        m_point_cloud_msg.points.emplace_back(point);
    }
}

void MB_1r2t::parse_packet()
{
    switch (m_state) {
    case SYNC0:
        if (m_serial_device->read(&m_packet.sync_0, 1) == false) {
            break;
        }

        if (m_packet.sync_0 == SYNC_BYTE0) {
            m_state = SYNC1;
        }

        break;

    case SYNC1:
        if (m_serial_device->read(&m_packet.sync_1, 1) == false) {
            break;
        }

        if (m_packet.sync_1 == SYNC_BYTE1) {
            m_state = HEADER;
        } else {
            m_state = SYNC0;
        }

        break;

    case HEADER:
        if (m_serial_device->read(m_packet.data, 8) == false) {
            m_state = SYNC0;
            break;
        }

        m_packet.type = m_packet.data[0];
        m_packet.data_length = m_packet.data[1];
        m_packet.start_angle = m_packet.data[3] << 8 | m_packet.data[2];
        m_packet.stop_angle = m_packet.data[5] << 8 | m_packet.data[4];
        m_packet.crc = m_packet.data[7] << 8 | m_packet.data[6];

        m_state = DATA;
        break;

    case DATA: {
        uint16_t bytes_to_read = m_packet.data_length * 3;

        // invalid data
        if (bytes_to_read > DATA_SIZE) {
            m_state = SYNC0;
            break;
        }

        if (m_serial_device->read(m_packet.data, bytes_to_read) == false) {
            m_state = SYNC0;
            break;
        }

        if (m_packet.type & 0x01) {
            scan_done();
            // stored in units of 0.1 Hz
            uint8_t scan_freq = m_packet.type >> 1;
            RCLCPP_DEBUG(
                get_logger(), "Scanning frequency: %d.%d Hz",
                scan_freq / 10, scan_freq % 10
            );
        } else {
            scan_data();
            if (m_packet.type != SCAN_DATA) {
                RCLCPP_WARN(get_logger(), "Unexpected packet type: %02X", m_packet.type);
            }
        }

        m_state = SYNC0;
        break;
    }
    default:
        RCLCPP_ERROR(get_logger(), "Unknown state: %d", m_state);
        m_state = SYNC0;
    }
}
