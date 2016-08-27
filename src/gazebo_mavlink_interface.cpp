/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "gazebo_mavlink_interface.h"
#include "geo_mag_declination.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkInterface);

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboMavlinkInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
                           motor_velocity_reference_pub_topic_);
  getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  getSdfParam<std::string>(_sdf, "lidarSubTopic", lidar_sub_topic_, lidar_sub_topic_);
  getSdfParam<std::string>(_sdf, "opticalFlowSubTopic",
      opticalFlow_sub_topic_, opticalFlow_sub_topic_);
  getSdfParam<std::string>(_sdf, "gpsSubTopic",
      gps_sub_topic_, gps_sub_topic_);
  getSdfParam<std::string>(_sdf, "magSubTopic",
      mag_sub_topic_, mag_sub_topic_);
  getSdfParam<std::string>(_sdf, "altSubTopic",
      alt_sub_topic_, alt_sub_topic_);

  joints_.resize(n_out_max);

  if (_sdf->HasElement("control_channels")) {
    sdf::ElementPtr cmapping = _sdf->GetElement("control_channels");

    // sdf::ElementPtr_V children = cmapping->elements;

    // for (unsigned i = 0; i < children.size(); i++) {
    //   std::string name = children.at(i).Get<std::string>();

    //   if (name == std::string("channel")) {
    //     //int input_index = _sdf->GetElement("input_index")->Get<int>();
    //     input_offset[i] = _sdf->GetElement("input_offset")->Get<double>();
    //     input_scaling[i] = _sdf->GetElement("input_scaling")->Get<double>();
    //   }
    // }
  }

  if (_sdf->HasElement("left_elevon_joint")) {
    std::string left_elevon_joint_name = _sdf->GetElement("left_elevon_joint")->Get<std::string>();
    left_elevon_joint_ = model_->GetJoint(left_elevon_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("left_elevon_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = left_elevon_joint_;
    }
  }

  if (_sdf->HasElement("left_aileron_joint")) {
    std::string left_elevon_joint_name = _sdf->GetElement("left_aileron_joint")->Get<std::string>();
    left_elevon_joint_ = model_->GetJoint(left_elevon_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("left_aileron_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = left_elevon_joint_;
    }
  }

  if (_sdf->HasElement("right_elevon_joint")) {
    std::string right_elevon_joint_name = _sdf->GetElement("right_elevon_joint")->Get<std::string>();
    right_elevon_joint_ = model_->GetJoint(right_elevon_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("right_elevon_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = right_elevon_joint_;
    }
  }

  if (_sdf->HasElement("right_aileron_joint")) {
    std::string right_elevon_joint_name = _sdf->GetElement("right_aileron_joint")->Get<std::string>();
    right_elevon_joint_ = model_->GetJoint(right_elevon_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("right_aileron_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = right_elevon_joint_;
    }
  }

  if (_sdf->HasElement("elevator_joint")) {
    std::string elevator_joint_name = _sdf->GetElement("elevator_joint")->Get<std::string>();
    elevator_joint_ = model_->GetJoint(elevator_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("elevator_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = elevator_joint_;
    }
  }

  if (_sdf->HasElement("propeller_joint")) {
    std::string propeller_joint_name = _sdf->GetElement("propeller_joint")->Get<std::string>();
    propeller_joint_ = model_->GetJoint(propeller_joint_name);
  }

  if (_sdf->HasElement("cgo3_mount_joint")) {
    std::string gimbal_yaw_joint_name = _sdf->GetElement("cgo3_mount_joint")->Get<std::string>();
    gimbal_yaw_joint_ = model_->GetJoint(gimbal_yaw_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("cgo3_mount_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = gimbal_yaw_joint_;
    }
  }

  if (_sdf->HasElement("cgo3_vertical_arm_joint")) {
    std::string gimbal_roll_joint_name = _sdf->GetElement("cgo3_vertical_arm_joint")->Get<std::string>();
    gimbal_roll_joint_ = model_->GetJoint(gimbal_roll_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("cgo3_vertical_arm_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = gimbal_roll_joint_;
    }
  }

  if (_sdf->HasElement("cgo3_horizontal_arm_joint")) {
    std::string gimbal_pitch_joint_name = _sdf->GetElement("cgo3_horizontal_arm_joint")->Get<std::string>();
    gimbal_pitch_joint_ = model_->GetJoint(gimbal_pitch_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("cgo3_horizontal_arm_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = gimbal_pitch_joint_;
    }
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  // Subscriber to IMU sensor_msgs::Imu Message and SITL message
  imu_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + imu_sub_topic_, &GazeboMavlinkInterface::IMUCallback, this);
  lidar_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + lidar_sub_topic_, &GazeboMavlinkInterface::LidarCallback, this);
  opticalFlow_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + opticalFlow_sub_topic_, &GazeboMavlinkInterface::OpticalFlowCallback, this);
  gps_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + gps_sub_topic_, &GazeboMavlinkInterface::GpsCallback, this);
  mag_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + mag_sub_topic_, &GazeboMavlinkInterface::MagCallback, this);
 
  // Publish gazebo's motor_speed message
  motor_velocity_reference_pub_ = node_handle_->Advertise<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + motor_velocity_reference_pub_topic_, 1);

  _rotor_count = 5;
  last_time_ = world_->GetSimTime();

  gravity_W_ = world_->GetPhysicsEngine()->GetGravity();

  //Create socket
  // udp socket data
  mavlink_addr_ = htonl(INADDR_ANY);
  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr = _sdf->GetElement("mavlink_addr")->Get<std::string>();
    if (mavlink_addr != "INADDR_ANY") {
      mavlink_addr_ = inet_addr(mavlink_addr.c_str());
      if (mavlink_addr_ == INADDR_NONE) {
        fprintf(stderr, "invalid mavlink_addr \"%s\"\n", mavlink_addr.c_str());
        return;
      }
    }
  }
  if (_sdf->HasElement("mavlink_udp_port")) {
    mavlink_udp_port_ = _sdf->GetElement("mavlink_udp_port")->Get<int>();
  }

  // try to setup udp socket for communcation with simulator
  if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    printf("create socket failed\n");
    return;
  }

  memset((char *)&_myaddr, 0, sizeof(_myaddr));
  _myaddr.sin_family = AF_INET;
  _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  // Let the OS pick the port
  _myaddr.sin_port = htons(0);

  if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
    printf("bind failed\n");
    return;
  }

  _srcaddr.sin_family = AF_INET;
  _srcaddr.sin_addr.s_addr = mavlink_addr_;
  _srcaddr.sin_port = htons(mavlink_udp_port_);
  _addrlen = sizeof(_srcaddr);

  fds[0].fd = _fd;
  fds[0].events = POLLIN;
}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {

  pollForMAVLinkMessages();

  common::Time now = world_->GetSimTime();

  if(received_first_referenc_) {

    mav_msgs::msgs::CommandMotorSpeed turning_velocities_msg;

    for (int i = 0; i < input_reference_.size(); i++){
      if (last_actuator_time_ == 0 || (now - last_actuator_time_).Double() > 0.2) {
        turning_velocities_msg.add_motor_speed(0);
      } else {
        turning_velocities_msg.add_motor_speed(input_reference_[i]);
      }
    }
    // TODO Add timestamp and Header
    // turning_velocities_msg->header.stamp.sec = now.sec;
    // turning_velocities_msg->header.stamp.nsec = now.nsec;

    // gzerr << turning_velocities_msg.motor_speed(0) << "\n";
    motor_velocity_reference_pub_->Publish(turning_velocities_msg);
  }
}

void GazeboMavlinkInterface::send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID) {
  component_ID = 0;
  uint8_t payload_len = mavlink_message_lengths[msgid];
  unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  /* header */
  buf[0] = MAVLINK_STX;
  buf[1] = payload_len;
  /* no idea which numbers should be here*/
  buf[2] = 100;
  buf[3] = 0;
  buf[4] = component_ID;
  buf[5] = msgid;

  /* payload */
  memcpy(&buf[MAVLINK_NUM_HEADER_BYTES],msg, payload_len);

  /* checksum */
  uint16_t checksum;
  crc_init(&checksum);
  crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
  crc_accumulate(mavlink_message_crcs[msgid], &checksum);

  buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
  buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

  ssize_t len;

  len = sendto(_fd, buf, packet_len, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));

  if (len <= 0) {
    printf("Failed sending mavlink message\n");
  }
}

void GazeboMavlinkInterface::IMUCallback(ConstIMUPtr& imu_message) {
  //math::Quaternion C_W_I;
  //C_W_I.w = imu_message->orientation().w();
  //C_W_I.x = imu_message->orientation().x();
  //C_W_I.y = imu_message->orientation().y();
  //C_W_I.z = imu_message->orientation().z();
  //math::Vector3 body_vel = C_W_I.RotateVectorReverse(model_->GetWorldLinearVel());

  mavlink_hil_sensor_t sensor_msg;
  sensor_msg.time_usec = imu_message->stamp().nsec()*1e3;
  sensor_msg.xacc = imu_message->linear_acceleration().x();
  sensor_msg.yacc = imu_message->linear_acceleration().y();
  sensor_msg.zacc = imu_message->linear_acceleration().z();
  sensor_msg.xgyro = imu_message->angular_velocity().x();
  sensor_msg.ygyro = imu_message->angular_velocity().y();
  sensor_msg.zgyro = imu_message->angular_velocity().z();
  sensor_msg.xmag = mag_I_.x;
  sensor_msg.ymag = mag_I_.y;
  sensor_msg.zmag = mag_I_.z;
  sensor_msg.abs_pressure = 0.0;
  //float q = 1.2754;
  //sensor_msg.diff_pressure = 0.5*q*(body_vel.z*body_vel.z + body_vel.x*body_vel.x) / 100;
  sensor_msg.diff_pressure = 0;
  sensor_msg.pressure_alt = pressure_alt_;
  sensor_msg.temperature = 0.0;
  sensor_msg.fields_updated = 4095;

  //gyro needed for optical flow message
  optflow_xgyro = imu_message->angular_velocity().x();
  optflow_ygyro = imu_message->angular_velocity().y();
  optflow_zgyro = imu_message->angular_velocity().z();

  send_mavlink_message(MAVLINK_MSG_ID_HIL_SENSOR, &sensor_msg, 200);    
}

void GazeboMavlinkInterface::LidarCallback(LidarPtr& lidar_message) {
  mavlink_distance_sensor_t sensor_msg;
  sensor_msg.time_boot_ms = lidar_message->time_msec();
  sensor_msg.min_distance = lidar_message->min_distance() * 100.0;
  sensor_msg.max_distance = lidar_message->max_distance() * 100.0;
  sensor_msg.current_distance = lidar_message->current_distance() * 100.0;
  sensor_msg.type = 0;
  sensor_msg.id = 0;
  sensor_msg.orientation = 0;
  sensor_msg.covariance = 0;

  //distance needed for optical flow message
  optflow_distance = lidar_message->current_distance(); //[m]

  send_mavlink_message(MAVLINK_MSG_ID_DISTANCE_SENSOR, &sensor_msg, 200);

}

void GazeboMavlinkInterface::OpticalFlowCallback(OpticalFlowPtr& opticalFlow_message) {
  mavlink_hil_optical_flow_t sensor_msg;
  sensor_msg.time_usec = opticalFlow_message->time_usec();
  sensor_msg.sensor_id = opticalFlow_message->sensor_id();
  sensor_msg.integration_time_us = opticalFlow_message->integration_time_us();
  sensor_msg.integrated_x = opticalFlow_message->integrated_x();
  sensor_msg.integrated_y = opticalFlow_message->integrated_y();
  sensor_msg.integrated_xgyro = optflow_ygyro * opticalFlow_message->integration_time_us() / 1000000.0; //xy switched
  sensor_msg.integrated_ygyro = optflow_xgyro * opticalFlow_message->integration_time_us() / 1000000.0; //xy switched
  sensor_msg.integrated_zgyro = -optflow_zgyro * opticalFlow_message->integration_time_us() / 1000000.0; //change direction
  sensor_msg.temperature = opticalFlow_message->temperature();
  sensor_msg.quality = opticalFlow_message->quality();
  sensor_msg.time_delta_distance_us = opticalFlow_message->time_delta_distance_us();
  sensor_msg.distance = optflow_distance;

  send_mavlink_message(MAVLINK_MSG_ID_HIL_OPTICAL_FLOW, &sensor_msg, 200);
}

void GazeboMavlinkInterface::GpsCallback(ConstGPSPtr& gz_msg) {
  float rad2deg = 180.0/M_PI;
  float speed = gz_msg->velocity_north()*gz_msg->velocity_north() + 
    gz_msg->velocity_east()*gz_msg->velocity_east() + 
    gz_msg->velocity_up()*gz_msg->velocity_up(); 
  float cog = atan2(gz_msg->velocity_east(), gz_msg->velocity_north());

  mavlink_hil_gps_t mav_msg;
  mav_msg.time_usec = gz_msg->time().nsec()*1e3;
  mav_msg.fix_type = 3;
  mav_msg.lat = gz_msg->latitude_deg() * 1e7;
  mav_msg.lon = gz_msg->longitude_deg() * 1e7;
  mav_msg.alt = gz_msg->altitude() * 1000;
  mav_msg.eph = 100;
  mav_msg.epv = 100;
  mav_msg.vel = speed * 100;
  mav_msg.vn = gz_msg->velocity_north() * 100;
  mav_msg.ve = gz_msg->velocity_east() * 100;
  mav_msg.vd = -gz_msg->velocity_up() * 100;
  mav_msg.cog = cog * rad2deg * 100;
  mav_msg.satellites_visible = 10;

  send_mavlink_message(MAVLINK_MSG_ID_HIL_GPS, &mav_msg, 200);
}

void GazeboMavlinkInterface::MagCallback(ConstMagnetometerPtr& gz_msg) {
  mag_I_.x = gz_msg->field_tesla().x();
  mag_I_.y = gz_msg->field_tesla().y();
  mag_I_.z = gz_msg->field_tesla().z();
}

void GazeboMavlinkInterface::AltCallback(ConstAltimeterPtr& gz_msg) {
  pressure_alt_ = gz_msg->vertical_position();
}

void GazeboMavlinkInterface::pollForMAVLinkMessages()
{
  int len;
  ::poll(&fds[0], (sizeof(fds[0])/sizeof(fds[0])), 0);
  if (fds[0].revents & POLLIN) {
    len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, &_addrlen);
    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (int i = 0; i < len; ++i)
      {
        if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status))
        {
          // have a message, handle it
          handle_message(&msg);
        }
      }
    }
  }
}

void GazeboMavlinkInterface::handle_message(mavlink_message_t *msg)
{
  switch(msg->msgid) {
  case MAVLINK_MSG_ID_HIL_CONTROLS:
    mavlink_hil_controls_t controls;
    mavlink_msg_hil_controls_decode(msg, &controls);
    bool armed = false;

    if ((controls.mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0) {
      armed = true;
    }

    const unsigned n_out = sizeof(inputs.control) / sizeof(inputs.control[0]);
    const unsigned block_size = 8;

    unsigned off = (controls.nav_mode * block_size);

    // We only support 8 outputs so far, so we
    // ignore the second, third and fourth output

    inputs.control[off + 0] = controls.roll_ailerons;
    inputs.control[off + 1] = controls.pitch_elevator;
    inputs.control[off + 2] = controls.yaw_rudder;
    inputs.control[off + 3] = controls.throttle;
    inputs.control[off + 4] = controls.aux1;
    inputs.control[off + 5] = controls.aux2;
    inputs.control[off + 6] = controls.aux3;
    inputs.control[off + 7] = controls.aux4;

    // XXX setting this to anything higher than 8 results
    // in memory smashing, likely due to a hardcoded
    // motor count somewhere
    if (off + block_size > 8) {
      break;
    }

    bool is_vtol = (right_elevon_joint_ != nullptr);

    // Set all scalings

    // Initialize all outputs as motors
    // if joints are present these will be
    // reconfigured in the next step
    for (unsigned i = off; i < off + block_size; i++) {
      input_index[i] = i;

      // scaling values
      input_offset[i] = 1.0;

      // XXX this needs re-investigation regarding
      // the correct scaling for the correct motor
      // model
      input_scaling[i] = 550.0;
      zero_position_disarmed[i] = 0.0;
      zero_position_armed[i] = (is_vtol) ? 0.0 : 100.0;
    }

    if (is_vtol) {
      // Config for standard VTOL model

      // Fift motor
      input_index[off + 4] = off + 4;
      input_offset[off + 4] = 1.0;
      input_scaling[off + 4] = 1600;
      zero_position_disarmed[off + 4] = 0.0;
      zero_position_armed[off + 4] = 0.0;

      // Servos
      for (unsigned i = off + 5; i < off + block_size; i++) {
        // scaling values
        input_index[i] = i;
        input_offset[i] = 0.0;
        input_scaling[i] = 1.0;
        zero_position_disarmed[i] = 0.0;
        zero_position_armed[i] = 0.0;
      }
    }

    last_actuator_time_ = world_->GetSimTime();

    input_reference_.resize(n_out);

    // set rotor speeds
    for (int i = 0; i < input_reference_.size(); i++) {
      if (armed) {
        input_reference_[i] = (inputs.control[input_index[i]] + input_offset[i]) * input_scaling[i] + zero_position_armed[i];
      } else {
        input_reference_[i] = zero_position_disarmed[i];
      }
    }

    // set joint positions
    for (int i = 0; i < input_reference_.size(); i++) {
      if (joints_[i]) {
#if GAZEBO_MAJOR_VERSION >= 6
        joints_[i]->SetPosition(0, input_reference_[i]);
#else
        joints_[i]->SetAngle(0, input_reference_[i]);
#endif
      }
    }

    // legacy method, can eventually be replaced
    if (right_elevon_joint_ != NULL && left_elevon_joint_!= 0 && elevator_joint_ != 0) {
#if GAZEBO_MAJOR_VERSION >= 6
      left_elevon_joint_->SetPosition(0, input_reference_[5]);
      right_elevon_joint_->SetPosition(0, input_reference_[6]);
      elevator_joint_->SetPosition(0, input_reference_[7]);
#else
      left_elevon_joint_->SetAngle(0, input_reference_[5]);
      right_elevon_joint_->SetAngle(0, input_reference_[6]);
      elevator_joint_->SetAngle(0, input_reference_[7]);
#endif
    }

    received_first_referenc_ = true;
    break;
  }
}

}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
