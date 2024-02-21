#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                         0, 1, 0, -eye_pos[1], 
                         0, 0, 1, -eye_pos[2], 
                         0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    rotation_angle = rotation_angle / 180.0 * acos(-1);
    // Create the model matrix for rotating the triangle around the Z axis.
    Eigen::Matrix4f rotate;
    rotate << 
                    cosf(rotation_angle), -sinf(rotation_angle), 0.0, 0.0,
                    sinf(rotation_angle), cosf(rotation_angle), 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;
    model = rotate * model;
    // Then return it.
    return model;
}

//NOTE:eye_fov means eye field of view
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    float alpha = eye_fov / 180.0 * acos(-1);
    float n = -zNear, f = -zFar;
    float t = tan(alpha / 2.0) * abs(n), r = t * aspect_ratio;
    float b = -t, l = -r;

    Eigen::Matrix4f Mp_o, translate, scale;
    Mp_o << n, 0, 0, 0, 
                    0, n, 0, 0,
                    0, 0, n + f, -n * f,
                    0, 0, 1, 0;

    translate << 1, 0, 0, -(r + l) / 2.0,
                         0, 1, 0, -(t + b) / 2.0,
                         0, 0, 1, -(n + f) / 2.0,
                         0, 0, 0, 1;

    scale << 2.0 / (r - l), 0, 0, 0,
                   0, 2.0 / (t -b), 0, 0,
                   0, 0, 2.0 /(n - f), 0,
                   0, 0, 0, 1; 
    projection = scale * translate * Mp_o * projection;                       
    // Then return it.

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f temp_rotation = Eigen::Matrix3f::Identity();

    angle = angle / 180.0 * acos(-1);
    Eigen::Matrix3f cross_product_matrix;
    cross_product_matrix << 0, -axis.z(), axis.y(),
                                               axis.z(), 0, -axis.x(),
                                               -axis.y(), axis.x(), 0,
        
    temp_rotation =  cos(angle) * temp_rotation + (1 - cos(angle)) * axis * axis.transpose() 
                                 + sin(angle) * cross_product_matrix;

    rotation.block<3, 3>(0, 0) = temp_rotation;

    return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(0);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
