
namespace Kinect2.MultiKinects2BodyTracking.Client.TCPConnection
{
    //client types, KINECT = kinect client, ROBOT = user client
    public enum clientTypes { KINECT, DATA_PROCESSOR, ROBOT, UNKNOWN };

    //define upload commands
    public enum UploadCommands
    {
        Update_knect_data_in_Base64_format, //	Used by Kinect Client
        Update_fused_knect_data_in_Base64_format,
        Update_raw_data
    }

    //define download commands
    public enum DownloadCommands
    {
        Get_total_number_of_alive_Kinect_Client,
        Get_kinect_matrices,
        Get_all_kinect_images,
        Get_object_frame,
        Download_all_kinect_data_in_Base64_string_format,
        Download_fused_kinect_data_in_Base64_string_format,
        Get_total_number_of_faces_from_fused_kinect_data,
        Get_total_number_of_skeletons_from_fused_kinect_data,
        Get_full_face_parameters_from_fused_kinect_data_orientation_point,
        Get_full_face_parameters_from_fused_kinect_data_radian,
        Get_face_positions_from_fused_kinect_data,
        Get_face_orientation_point_from_fused_kinect_data,
        Get_face_orientation_angle_from_fused_kinect_data,
        Get_skeleton_position_array_from_fused_kinect_data,
        Get_full_skeleton_array_from_fused_kinect_data,
        Get_upper_body_joint_information_of_skeleton_array_from_fused_kinect_data,
        Get_head_joint_information_of_skeleton_array_from_fused_kinect_data,
        Get_sound_parameters_from_a_specified_kinect,
        Get_full_face_parameters_from_a_specified_kinect_rotation_in_orientation_point,
        Get_full_face_parameters_from_a_specified_kinect_rotation_in_radian,
        Get_face_positions_from_a_specified_kinect,
        Get_face_orientation_point_from_a_specified_kinect,
        Get_face_orientation_angle_from_a_specified_kinect,
        Get_skeleton_position_array_from_a_specified_kinect,
        Get_full_skeleton_array_from_a_specified_kinect,
        Get_upper_body_joint_information_of_skeleton_array_from_a_specified_kinect,
        Get_head_joint_information_of_skeleton_array_from_a_specified_kinect

    }

    //define other commands
    public enum OtherCommands
    {
        Disconnect_from_server_server_close_connection,
        Disconnect_from_server_server_hangs_and_wait_for_this_client_to_close_connection
    }
}