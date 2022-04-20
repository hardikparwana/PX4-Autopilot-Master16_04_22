

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/ucdr/timesync.h>
#include <uORB/ucdr/trajectory_waypoint.h>
#include <uORB/ucdr/vehicle_control_mode.h>
#include <uORB/ucdr/vehicle_odometry.h>
#include <uORB/ucdr/vehicle_status.h>
#include <uORB/ucdr/collision_constraints.h>
#include <uORB/ucdr/timesync_status.h>
#include <uORB/ucdr/sensor_combined.h>
#include <uORB/ucdr/vehicle_local_position.h>
#include <uORB/ucdr/vehicle_angular_velocity.h>
#include <uORB/ucdr/vehicle_angular_acceleration.h>
#include <uORB/ucdr/vehicle_trajectory_waypoint.h>
#include <uORB/ucdr/debug_array.h>
#include <uORB/ucdr/debug_key_value.h>
#include <uORB/ucdr/debug_value.h>
#include <uORB/ucdr/debug_vect.h>
#include <uORB/ucdr/offboard_control_mode.h>
#include <uORB/ucdr/optical_flow.h>
#include <uORB/ucdr/position_setpoint.h>
#include <uORB/ucdr/position_setpoint_triplet.h>
#include <uORB/ucdr/telemetry_status.h>
#include <uORB/ucdr/timesync.h>
#include <uORB/ucdr/vehicle_command.h>
#include <uORB/ucdr/vehicle_local_position_setpoint.h>
#include <uORB/ucdr/vehicle_trajectory_waypoint.h>
#include <uORB/ucdr/onboard_computer_status.h>
#include <uORB/ucdr/trajectory_bezier.h>
#include <uORB/ucdr/vehicle_trajectory_bezier.h>
#include <uORB/ucdr/actuator_controls.h>
#include <uORB/ucdr/vehicle_local_position_setpoint.h>
#include <uORB/ucdr/vehicle_odometry.h>
#include <uORB/ucdr/vehicle_odometry.h>

// Subscribers for messages to send
struct SendTopicsSubs {
	uORB::Subscription timesync_sub{ORB_ID(timesync)};
	uxrObjectId timesync_data_writer;
	uORB::Subscription trajectory_waypoint_sub{ORB_ID(trajectory_waypoint)};
	uxrObjectId trajectory_waypoint_data_writer;
	uORB::Subscription vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uxrObjectId vehicle_control_mode_data_writer;
	uORB::Subscription vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uxrObjectId vehicle_odometry_data_writer;
	uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
	uxrObjectId vehicle_status_data_writer;
	uORB::Subscription collision_constraints_sub{ORB_ID(collision_constraints)};
	uxrObjectId collision_constraints_data_writer;
	uORB::Subscription timesync_status_sub{ORB_ID(timesync_status)};
	uxrObjectId timesync_status_data_writer;
	uORB::Subscription sensor_combined_sub{ORB_ID(sensor_combined)};
	uxrObjectId sensor_combined_data_writer;
	uORB::Subscription vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uxrObjectId vehicle_local_position_data_writer;
	uORB::Subscription vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uxrObjectId vehicle_angular_velocity_data_writer;
	uORB::Subscription vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)};
	uxrObjectId vehicle_angular_acceleration_data_writer;
	uORB::Subscription vehicle_trajectory_waypoint_desired_sub{ORB_ID(vehicle_trajectory_waypoint_desired)};
	uxrObjectId vehicle_trajectory_waypoint_desired_data_writer;

	uxrSession* session;

	uint32_t num_payload_sent{};

	bool init(uxrSession* session_, uxrStreamId stream_id, uxrObjectId participant_id);
	void update(uxrStreamId stream_id);
};

bool SendTopicsSubs::init(uxrSession* session_, uxrStreamId stream_id, uxrObjectId participant_id)
{
	session = session_;

	{

		uxrObjectId topic_id = uxr_object_id(0+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/Timesync</name>"
				"<dataType>px4_msgs::msg::dds_::Timesync_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(0+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(0+1, UXR_DATAWRITER_ID);
		timesync_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/Timesync</name>"
				"<dataType>px4_msgs::msg::dds_::Timesync_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "Timesync", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(1+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/TrajectoryWaypoint</name>"
				"<dataType>px4_msgs::msg::dds_::TrajectoryWaypoint_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(1+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(1+1, UXR_DATAWRITER_ID);
		trajectory_waypoint_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/TrajectoryWaypoint</name>"
				"<dataType>px4_msgs::msg::dds_::TrajectoryWaypoint_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "TrajectoryWaypoint", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(2+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/VehicleControlMode</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleControlMode_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(2+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(2+1, UXR_DATAWRITER_ID);
		vehicle_control_mode_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/VehicleControlMode</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleControlMode_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "VehicleControlMode", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(3+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/VehicleOdometry</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleOdometry_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(3+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(3+1, UXR_DATAWRITER_ID);
		vehicle_odometry_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/VehicleOdometry</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleOdometry_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "VehicleOdometry", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(4+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/VehicleStatus</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleStatus_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(4+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(4+1, UXR_DATAWRITER_ID);
		vehicle_status_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/VehicleStatus</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleStatus_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "VehicleStatus", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(5+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/CollisionConstraints</name>"
				"<dataType>px4_msgs::msg::dds_::CollisionConstraints_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(5+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(5+1, UXR_DATAWRITER_ID);
		collision_constraints_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/CollisionConstraints</name>"
				"<dataType>px4_msgs::msg::dds_::CollisionConstraints_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "CollisionConstraints", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(6+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/TimesyncStatus</name>"
				"<dataType>px4_msgs::msg::dds_::TimesyncStatus_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(6+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(6+1, UXR_DATAWRITER_ID);
		timesync_status_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/TimesyncStatus</name>"
				"<dataType>px4_msgs::msg::dds_::TimesyncStatus_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "TimesyncStatus", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(7+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/SensorCombined</name>"
				"<dataType>px4_msgs::msg::dds_::SensorCombined_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(7+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(7+1, UXR_DATAWRITER_ID);
		sensor_combined_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/SensorCombined</name>"
				"<dataType>px4_msgs::msg::dds_::SensorCombined_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "SensorCombined", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(8+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/VehicleLocalPosition</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleLocalPosition_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(8+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(8+1, UXR_DATAWRITER_ID);
		vehicle_local_position_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/VehicleLocalPosition</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleLocalPosition_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "VehicleLocalPosition", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(9+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/VehicleAngularVelocity</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleAngularVelocity_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(9+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(9+1, UXR_DATAWRITER_ID);
		vehicle_angular_velocity_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/VehicleAngularVelocity</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleAngularVelocity_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "VehicleAngularVelocity", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(10+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/VehicleAngularAcceleration</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleAngularAcceleration_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(10+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(10+1, UXR_DATAWRITER_ID);
		vehicle_angular_acceleration_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/VehicleAngularAcceleration</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleAngularAcceleration_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "VehicleAngularAcceleration", status[0], status[1], status[2]);
			return false;
		}
	}

	{

		uxrObjectId topic_id = uxr_object_id(11+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/VehicleTrajectoryWaypointDesired</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleTrajectoryWaypointDesired_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(11+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(11+1, UXR_DATAWRITER_ID);
		vehicle_trajectory_waypoint_desired_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/VehicleTrajectoryWaypointDesired</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleTrajectoryWaypointDesired_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "VehicleTrajectoryWaypointDesired", status[0], status[1], status[2]);
			return false;
		}
	}


	return true;
}

void SendTopicsSubs::update(uxrStreamId stream_id)
{
	{
		timesync_s data;

		if (timesync_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_timesync();
			uxr_prepare_output_stream(session, stream_id, timesync_data_writer, &ub, topic_size);
			ucdr_serialize_timesync(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		trajectory_waypoint_s data;

		if (trajectory_waypoint_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_trajectory_waypoint();
			uxr_prepare_output_stream(session, stream_id, trajectory_waypoint_data_writer, &ub, topic_size);
			ucdr_serialize_trajectory_waypoint(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		vehicle_control_mode_s data;

		if (vehicle_control_mode_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_vehicle_control_mode();
			uxr_prepare_output_stream(session, stream_id, vehicle_control_mode_data_writer, &ub, topic_size);
			ucdr_serialize_vehicle_control_mode(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		vehicle_odometry_s data;

		if (vehicle_odometry_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_vehicle_odometry();
			uxr_prepare_output_stream(session, stream_id, vehicle_odometry_data_writer, &ub, topic_size);
			ucdr_serialize_vehicle_odometry(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		vehicle_status_s data;

		if (vehicle_status_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_vehicle_status();
			uxr_prepare_output_stream(session, stream_id, vehicle_status_data_writer, &ub, topic_size);
			ucdr_serialize_vehicle_status(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		collision_constraints_s data;

		if (collision_constraints_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_collision_constraints();
			uxr_prepare_output_stream(session, stream_id, collision_constraints_data_writer, &ub, topic_size);
			ucdr_serialize_collision_constraints(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		timesync_status_s data;

		if (timesync_status_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_timesync_status();
			uxr_prepare_output_stream(session, stream_id, timesync_status_data_writer, &ub, topic_size);
			ucdr_serialize_timesync_status(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		sensor_combined_s data;

		if (sensor_combined_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_sensor_combined();
			uxr_prepare_output_stream(session, stream_id, sensor_combined_data_writer, &ub, topic_size);
			ucdr_serialize_sensor_combined(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		vehicle_local_position_s data;

		if (vehicle_local_position_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_vehicle_local_position();
			uxr_prepare_output_stream(session, stream_id, vehicle_local_position_data_writer, &ub, topic_size);
			ucdr_serialize_vehicle_local_position(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		vehicle_angular_velocity_s data;

		if (vehicle_angular_velocity_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_vehicle_angular_velocity();
			uxr_prepare_output_stream(session, stream_id, vehicle_angular_velocity_data_writer, &ub, topic_size);
			ucdr_serialize_vehicle_angular_velocity(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		vehicle_angular_acceleration_s data;

		if (vehicle_angular_acceleration_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_vehicle_angular_acceleration();
			uxr_prepare_output_stream(session, stream_id, vehicle_angular_acceleration_data_writer, &ub, topic_size);
			ucdr_serialize_vehicle_angular_acceleration(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
	{
		vehicle_trajectory_waypoint_s data;

		if (vehicle_trajectory_waypoint_desired_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_vehicle_trajectory_waypoint();
			uxr_prepare_output_stream(session, stream_id, vehicle_trajectory_waypoint_desired_data_writer, &ub, topic_size);
			ucdr_serialize_vehicle_trajectory_waypoint(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}

}

static void on_topic_update(uxrSession* session, uxrObjectId object_id,
	uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t
	length, void* args);

// Publishers for received messages
struct RcvTopicsPubs {
	uORB::Publication <debug_array_s> debug_array_pub{ORB_ID(debug_array)};
	uORB::Publication <debug_key_value_s> debug_key_value_pub{ORB_ID(debug_key_value)};
	uORB::Publication <debug_value_s> debug_value_pub{ORB_ID(debug_value)};
	uORB::Publication <debug_vect_s> debug_vect_pub{ORB_ID(debug_vect)};
	uORB::Publication <offboard_control_mode_s> offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication <optical_flow_s> optical_flow_pub{ORB_ID(optical_flow)};
	uORB::Publication <position_setpoint_s> position_setpoint_pub{ORB_ID(position_setpoint)};
	uORB::Publication <position_setpoint_triplet_s> position_setpoint_triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication <telemetry_status_s> telemetry_status_pub{ORB_ID(telemetry_status)};
	uORB::Publication <timesync_s> timesync_pub{ORB_ID(timesync)};
	uORB::Publication <vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication <vehicle_local_position_setpoint_s> vehicle_local_position_setpoint_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication <vehicle_trajectory_waypoint_s> vehicle_trajectory_waypoint_pub{ORB_ID(vehicle_trajectory_waypoint)};
	uORB::Publication <onboard_computer_status_s> onboard_computer_status_pub{ORB_ID(onboard_computer_status)};
	uORB::Publication <trajectory_bezier_s> trajectory_bezier_pub{ORB_ID(trajectory_bezier)};
	uORB::Publication <vehicle_trajectory_bezier_s> vehicle_trajectory_bezier_pub{ORB_ID(vehicle_trajectory_bezier)};
	uORB::Publication <actuator_controls_s> actuator_controls_pub{ORB_ID(actuator_controls)};
	uORB::Publication <vehicle_local_position_setpoint_s> trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication <vehicle_odometry_s> vehicle_mocap_odometry_pub{ORB_ID(vehicle_mocap_odometry)};
	uORB::Publication <vehicle_odometry_s> vehicle_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};

	uxrSession* session;

	uint32_t num_payload_received{};

	bool init(uxrSession* session_, uxrStreamId stream_id, uxrStreamId input_stream, uxrObjectId participant_id);
};

bool RcvTopicsPubs::init(uxrSession* session_, uxrStreamId stream_id, uxrStreamId input_stream, uxrObjectId participant_id)
{
	session = session_;
    uxr_set_topic_callback(session, on_topic_update, this);


	{

		uxrObjectId subscriber_id = uxr_object_id(0+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+0, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/DebugArray</name>"
				"<dataType>px4_msgs::msg::dds_::DebugArray_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(0+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/DebugArray</name>"
												 "<dataType>px4_msgs::msg::dds_::DebugArray_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "debug_array", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(1+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/DebugKeyValue</name>"
				"<dataType>px4_msgs::msg::dds_::DebugKeyValue_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(1+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/DebugKeyValue</name>"
												 "<dataType>px4_msgs::msg::dds_::DebugKeyValue_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "debug_key_value", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(2+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+2, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/DebugValue</name>"
				"<dataType>px4_msgs::msg::dds_::DebugValue_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(2+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/DebugValue</name>"
												 "<dataType>px4_msgs::msg::dds_::DebugValue_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "debug_value", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(3+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+3, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/DebugVect</name>"
				"<dataType>px4_msgs::msg::dds_::DebugVect_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(3+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/DebugVect</name>"
												 "<dataType>px4_msgs::msg::dds_::DebugVect_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "debug_vect", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(4+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+4, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/OffboardControlMode</name>"
				"<dataType>px4_msgs::msg::dds_::OffboardControlMode_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(4+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/OffboardControlMode</name>"
												 "<dataType>px4_msgs::msg::dds_::OffboardControlMode_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "offboard_control_mode", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(5+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+5, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/OpticalFlow</name>"
				"<dataType>px4_msgs::msg::dds_::OpticalFlow_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(5+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/OpticalFlow</name>"
												 "<dataType>px4_msgs::msg::dds_::OpticalFlow_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "optical_flow", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(6+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+6, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/PositionSetpoint</name>"
				"<dataType>px4_msgs::msg::dds_::PositionSetpoint_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(6+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/PositionSetpoint</name>"
												 "<dataType>px4_msgs::msg::dds_::PositionSetpoint_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "position_setpoint", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(7+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+7, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/PositionSetpointTriplet</name>"
				"<dataType>px4_msgs::msg::dds_::PositionSetpointTriplet_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(7+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/PositionSetpointTriplet</name>"
												 "<dataType>px4_msgs::msg::dds_::PositionSetpointTriplet_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "position_setpoint_triplet", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(8+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+8, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/TelemetryStatus</name>"
				"<dataType>px4_msgs::msg::dds_::TelemetryStatus_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(8+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/TelemetryStatus</name>"
												 "<dataType>px4_msgs::msg::dds_::TelemetryStatus_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "telemetry_status", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(9+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+9, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/Timesync</name>"
				"<dataType>px4_msgs::msg::dds_::Timesync_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(9+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/Timesync</name>"
												 "<dataType>px4_msgs::msg::dds_::Timesync_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "timesync", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(10+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+10, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/VehicleCommand</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleCommand_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(10+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/VehicleCommand</name>"
												 "<dataType>px4_msgs::msg::dds_::VehicleCommand_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "vehicle_command", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(11+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+11, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/VehicleLocalPositionSetpoint</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleLocalPositionSetpoint_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(11+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/VehicleLocalPositionSetpoint</name>"
												 "<dataType>px4_msgs::msg::dds_::VehicleLocalPositionSetpoint_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "vehicle_local_position_setpoint", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(12+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+12, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/VehicleTrajectoryWaypoint</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleTrajectoryWaypoint_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(12+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/VehicleTrajectoryWaypoint</name>"
												 "<dataType>px4_msgs::msg::dds_::VehicleTrajectoryWaypoint_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "vehicle_trajectory_waypoint", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(13+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+13, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/OnboardComputerStatus</name>"
				"<dataType>px4_msgs::msg::dds_::OnboardComputerStatus_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(13+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/OnboardComputerStatus</name>"
												 "<dataType>px4_msgs::msg::dds_::OnboardComputerStatus_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "onboard_computer_status", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(14+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+14, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/TrajectoryBezier</name>"
				"<dataType>px4_msgs::msg::dds_::TrajectoryBezier_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(14+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/TrajectoryBezier</name>"
												 "<dataType>px4_msgs::msg::dds_::TrajectoryBezier_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "trajectory_bezier", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(15+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+15, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/VehicleTrajectoryBezier</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleTrajectoryBezier_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(15+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/VehicleTrajectoryBezier</name>"
												 "<dataType>px4_msgs::msg::dds_::VehicleTrajectoryBezier_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "vehicle_trajectory_bezier", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(16+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+16, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/ActuatorControls</name>"
				"<dataType>px4_msgs::msg::dds_::ActuatorControls_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(16+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/ActuatorControls</name>"
												 "<dataType>px4_msgs::msg::dds_::ActuatorControls_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "actuator_controls", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(17+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+17, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/TrajectorySetpoint</name>"
				"<dataType>px4_msgs::msg::dds_::TrajectorySetpoint_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(17+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/TrajectorySetpoint</name>"
												 "<dataType>px4_msgs::msg::dds_::TrajectorySetpoint_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "trajectory_setpoint", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(18+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+18, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/VehicleMocapOdometry</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleMocapOdometry_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(18+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/VehicleMocapOdometry</name>"
												 "<dataType>px4_msgs::msg::dds_::VehicleMocapOdometry_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "vehicle_mocap_odometry", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

	{

		uxrObjectId subscriber_id = uxr_object_id(19+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+19, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/VehicleVisualOdometry</name>"
				"<dataType>px4_msgs::msg::dds_::VehicleVisualOdometry_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(19+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/VehicleVisualOdometry</name>"
												 "<dataType>px4_msgs::msg::dds_::VehicleVisualOdometry_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "vehicle_visual_odometry", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}


	return true;
}

void on_topic_update(uxrSession* session, uxrObjectId object_id,
	uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t
	length, void* args)
{
	RcvTopicsPubs* pubs = (RcvTopicsPubs*)args;
	pubs->num_payload_received += length;

	switch (object_id.id) {
		case 0+1: {
			debug_array_s topic;
			if (ucdr_deserialize_debug_array(*ub, topic)) {
				pubs->debug_array_pub.publish(topic);
			}
		}
		break;
		case 1+1: {
			debug_key_value_s topic;
			if (ucdr_deserialize_debug_key_value(*ub, topic)) {
				pubs->debug_key_value_pub.publish(topic);
			}
		}
		break;
		case 2+1: {
			debug_value_s topic;
			if (ucdr_deserialize_debug_value(*ub, topic)) {
				pubs->debug_value_pub.publish(topic);
			}
		}
		break;
		case 3+1: {
			debug_vect_s topic;
			if (ucdr_deserialize_debug_vect(*ub, topic)) {
				pubs->debug_vect_pub.publish(topic);
			}
		}
		break;
		case 4+1: {
			offboard_control_mode_s topic;
			if (ucdr_deserialize_offboard_control_mode(*ub, topic)) {
				pubs->offboard_control_mode_pub.publish(topic);
			}
		}
		break;
		case 5+1: {
			optical_flow_s topic;
			if (ucdr_deserialize_optical_flow(*ub, topic)) {
				pubs->optical_flow_pub.publish(topic);
			}
		}
		break;
		case 6+1: {
			position_setpoint_s topic;
			if (ucdr_deserialize_position_setpoint(*ub, topic)) {
				pubs->position_setpoint_pub.publish(topic);
			}
		}
		break;
		case 7+1: {
			position_setpoint_triplet_s topic;
			if (ucdr_deserialize_position_setpoint_triplet(*ub, topic)) {
				pubs->position_setpoint_triplet_pub.publish(topic);
			}
		}
		break;
		case 8+1: {
			telemetry_status_s topic;
			if (ucdr_deserialize_telemetry_status(*ub, topic)) {
				pubs->telemetry_status_pub.publish(topic);
			}
		}
		break;
		case 9+1: {
			timesync_s topic;
			if (ucdr_deserialize_timesync(*ub, topic)) {
				pubs->timesync_pub.publish(topic);
			}
		}
		break;
		case 10+1: {
			vehicle_command_s topic;
			if (ucdr_deserialize_vehicle_command(*ub, topic)) {
				pubs->vehicle_command_pub.publish(topic);
			}
		}
		break;
		case 11+1: {
			vehicle_local_position_setpoint_s topic;
			if (ucdr_deserialize_vehicle_local_position_setpoint(*ub, topic)) {
				pubs->vehicle_local_position_setpoint_pub.publish(topic);
			}
		}
		break;
		case 12+1: {
			vehicle_trajectory_waypoint_s topic;
			if (ucdr_deserialize_vehicle_trajectory_waypoint(*ub, topic)) {
				pubs->vehicle_trajectory_waypoint_pub.publish(topic);
			}
		}
		break;
		case 13+1: {
			onboard_computer_status_s topic;
			if (ucdr_deserialize_onboard_computer_status(*ub, topic)) {
				pubs->onboard_computer_status_pub.publish(topic);
			}
		}
		break;
		case 14+1: {
			trajectory_bezier_s topic;
			if (ucdr_deserialize_trajectory_bezier(*ub, topic)) {
				pubs->trajectory_bezier_pub.publish(topic);
			}
		}
		break;
		case 15+1: {
			vehicle_trajectory_bezier_s topic;
			if (ucdr_deserialize_vehicle_trajectory_bezier(*ub, topic)) {
				pubs->vehicle_trajectory_bezier_pub.publish(topic);
			}
		}
		break;
		case 16+1: {
			actuator_controls_s topic;
			if (ucdr_deserialize_actuator_controls(*ub, topic)) {
				pubs->actuator_controls_pub.publish(topic);
			}
		}
		break;
		case 17+1: {
			vehicle_local_position_setpoint_s topic;
			if (ucdr_deserialize_vehicle_local_position_setpoint(*ub, topic)) {
				pubs->trajectory_setpoint_pub.publish(topic);
			}
		}
		break;
		case 18+1: {
			vehicle_odometry_s topic;
			if (ucdr_deserialize_vehicle_odometry(*ub, topic)) {
				pubs->vehicle_mocap_odometry_pub.publish(topic);
			}
		}
		break;
		case 19+1: {
			vehicle_odometry_s topic;
			if (ucdr_deserialize_vehicle_odometry(*ub, topic)) {
				pubs->vehicle_visual_odometry_pub.publish(topic);
			}
		}
		break;

		default:
		PX4_ERR("unknown object id: %i", object_id.id);
		break;
	}
}
