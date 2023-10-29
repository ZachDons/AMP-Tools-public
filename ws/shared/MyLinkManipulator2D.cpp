// #include "MyLinkManipulator2D.h"
// using namespace amp;

// MyLinkManipulator2D::MyLinkManipulator2D() : LinkManipulator2D()
// {
// }
// MyLinkManipulator2D::MyLinkManipulator2D(const std::vector<double> &link_lengths)
//     : LinkManipulator2D(link_lengths) {}
// MyLinkManipulator2D::MyLinkManipulator2D(const Eigen::Vector2d &base_location, const std::vector<double> &link_lengths)
//     : LinkManipulator2D(base_location, link_lengths) {}

// // Transformation matrix
// Eigen::Matrix3d Rot_Z(double angle_rad, double x_trans, double y_trans)
// {
//     // Define sin and cos
//     double cos_theta = std::cos(angle_rad);
//     double sin_theta = std::sin(angle_rad);
//     // Define the 2D rotation matrix
//     Eigen::Matrix3d rotation_matrix;
//     rotation_matrix << cos_theta, -sin_theta, x_trans,
//         sin_theta, cos_theta, y_trans,
//         0, 0, 1;
//     // Apply the rotation matrix to the vector
//     return rotation_matrix;
// }

// Eigen::Vector2d MyLinkManipulator2D::getJointLocation(const ManipulatorState &state,
//                                                       uint32_t joint_index) const
// {
//     // Initialize variables
//     vector<double> link_lengths = getLinkLengths();
//     Vector2d base_location_2d = getBaseLocation();
//     Eigen::Vector3d base_location;
//     base_location << base_location_2d, 1.0;

//     // sizes
//     int num_links = link_lengths.size();
//     // cout << "Link Len 1: " << link_lengths[0] << std::endl;
//     // cout << "Link Len 2: " << link_lengths[1] << std::endl;
//     // cout << "Link Len 3: " << link_lengths[2] << std::endl;
//     // cout << "State 1: " << state[0] << std::endl;
//     // cout << "State 2: " << state[1] << std::endl;
//     // cout << "State 3: " << state[2] << std::endl;

//     //  Preallocate
//     Eigen::Vector2d location;
//     // Perform transformations to get the desired joint location
//     if (joint_index == 0) // base location
//     {
//         location = {0.0, 0.0};
//     }
//     else if (joint_index == 1) // first joint
//     {
//         Eigen::Matrix3d T2 = Rot_Z(state[1], link_lengths[0], 0); // pose of A2 in A1 frame
//         Eigen::Matrix3d T1 = Rot_Z(state[0], 0, 0);               // pose of As in global frame
//         // cout << "T1: " << std::endl
//         //      << T1 << std::endl;
//         // cout << "T2: " << std::endl
//         //      << T2 << std::endl;
//         Eigen::Vector3d config_jt_1 = T1 * T2 * base_location;
//         // cout << "Joint config 1: " << config_jt_1 << std::endl;
//         location = config_jt_1.head<2>();
//     }
//     else if (joint_index == 2) // second joint
//     {
//         Eigen::Matrix3d T3 = Rot_Z(state[2], link_lengths[1], 0); // pose of A3 in A2 frame
//         Eigen::Matrix3d T2 = Rot_Z(state[1], link_lengths[0], 0); // pose of A2 in A1 frame
//         Eigen::Matrix3d T1 = Rot_Z(state[0], 0, 0);               // pose of As in global frame
//         Eigen::Vector3d config_jt_2 = T1 * T2 * T3 * base_location;
//         // cout << "T3: " << std::endl
//         //      << T3 << std::endl;
//         // cout << "Joint config 2: " << config_jt_2 << std::endl;
//         location = config_jt_2.head<2>();
//         // location = {1.0, 1.0};
//     }
//     else if (joint_index == 3) // end joint
//     {
//         Eigen::Matrix3d T4 = Rot_Z(0, link_lengths[2], 0);        // pose of end effector in A3 frame
//         Eigen::Matrix3d T3 = Rot_Z(state[2], link_lengths[1], 0); // pose of A3 in A2 frame
//         Eigen::Matrix3d T2 = Rot_Z(state[1], link_lengths[0], 0); // pose of A2 in A1 frame
//         Eigen::Matrix3d T1 = Rot_Z(state[0], 0, 0);               // pose of As in global frame
//         Eigen::Vector3d config_jt_3 = T1 * T2 * T3 * T4 * base_location;
//         // cout << "T4: " << std::endl
//         //      << T4 << std::endl;
//         // cout << "Joint config 3: " << config_jt_3 << std::endl;
//         location = config_jt_3.head<2>();
//         // location = {1.0, 1.0};
//     }
//     else
//     {
//         // cout << "Error(Zach): You asked for a joint index that is not defined" << std::endl;
//     }
//     return location;
// }

// ManipulatorState MyLinkManipulator2D::getConfigurationFromIK(const Eigen::Vector2d &end_effector_location) const
// {
//     // Initialize variables
//     vector<double> link_lengths = getLinkLengths();
//     double link_len_1 = link_lengths[0];
//     double link_len_2 = link_lengths[1];
//     double link_len_3 = link_lengths[2];
//     // End effector posittion
//     double d_x = end_effector_location.x();
//     double d_y = end_effector_location.y();
//     // Loop vars
//     const double twoPi = 2 * M_PI;
//     int num_angles = 36;
//     // Test theta 3 angles
//     for (int i_angle = 0; i_angle < num_angles; ++i_angle)
//     {
//         // test angle phi
//         double phi = i_angle * (twoPi / num_angles);
//         // End effector constraint
//         double xx = link_len_3 * std::cos(phi) + d_x;
//         double yy = link_len_3 * std::sin(phi) + d_y;
//         // Theta_2
//         double theta_2 = std::acos((((xx * xx) + (yy * yy)) - ((link_len_1 * link_len_1) + (link_len_2 * link_len_2))) / (2 * link_len_1 * link_len_2));
//         // Theta_1: only does the positive angle
//         double theta_1 = std::acos(((xx * (link_len_1 + link_len_2 * std::cos(theta_2))) + (yy * link_len_2 * std::sqrt(1 - (std::cos(theta_2) * std::cos(theta_2))))) / ((xx * xx) + (yy * yy)));
//         double theta_3 = phi - (theta_1 + theta_2 + M_PI);
//         cout << "IDX: " << i_angle << std::endl;
//         cout << "Theta 1: " << theta_1 << std::endl;
//         cout << "Theta 2: " << theta_2 << std::endl;
//         cout << "Theta 3: " << phi << std::endl;
//         cout << "Phi: " << phi << std::endl;

//         if ((!std::isnan(theta_1)) && (!std::isnan(theta_2)))
//         {
//             ManipulatorState selected_state = {theta_1, theta_2, theta_3};
//             return selected_state;
//         }
//     }
// }
