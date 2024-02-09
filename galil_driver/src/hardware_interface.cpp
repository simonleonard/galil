#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "galil_driver/hardware_interface.hpp"

namespace galil_driver {

  GalilSystemHardwareInterface::~GalilSystemHardwareInterface(){
    on_deactivate(rclcpp_lifecycle::State());
  }
  
  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_init(const hardware_interface::HardwareInfo& info){
    if( hardware_interface::SystemInterface::on_init(info) !=
	hardware_interface::CallbackReturn::SUCCESS ){
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints){
      for ( std::size_t i=0; i<joint.command_interfaces.size(); i++ ){
	if( joint.command_interfaces[i].name == hardware_interface::HW_IF_POSITION )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " has position command interface.");
	if( joint.command_interfaces[i].name == hardware_interface::HW_IF_VELOCITY )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " has velocity command interface.");
      }
      for ( std::size_t i=0; i<joint.state_interfaces.size(); i++ ){
	if( joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " has position state interface.");
	if( joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " has velocity state interface.");
	if( joint.state_interfaces[i].name == hardware_interface::HW_IF_EFFORT )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " has effort state interface.");
      }
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
    RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Configuring ...please wait...");

    GSize BUFFER_LENGTH=32;
    GSize bytes_returned;
    char buffer[32];
    std::string address("192.168.0.99");
    if( GOpen( address.c_str(), &connection ) == G_NO_ERROR ){
      RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Connection to Galil successful.");
    }
    else{
      RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to open connection with Galil.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    /*
    if( GCommand(connection, "DPA=0", buffer, BUFFER_LENGTH, &bytes_returned) != G_NO_ERROR ){
      std::cout << buffer << std::endl;
    }
    else{ std::cout << "error" << std::endl; }
    if( GCommand(connection, "DPB=0", buffer, BUFFER_LENGTH, &bytes_returned) != G_NO_ERROR ){
      std::cout << buffer << std::endl;
    }
    if( GCommand(connection, "DPC=0", buffer, BUFFER_LENGTH, &bytes_returned) != G_NO_ERROR ){
      std::cout << buffer << std::endl;
    }
    if( GCommand(connection, "PTA=1", buffer, BUFFER_LENGTH, &bytes_returned) != G_NO_ERROR ){
      std::cout << buffer << std::endl;
    }
    if( GCommand(connection, "PTB=1", buffer, BUFFER_LENGTH, &bytes_returned) != G_NO_ERROR ){
      std::cout << buffer << std::endl;
    }
    if( GCommand(connection, "PTC=1", buffer, BUFFER_LENGTH, &bytes_returned) != G_NO_ERROR ){
      std::cout << buffer << std::endl;
    }
    */
    
    RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  std::vector<hardware_interface::StateInterface> GalilSystemHardwareInterface::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++){
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
								       hardware_interface::HW_IF_POSITION,
								       &hw_states_position_[i]) );
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
								       hardware_interface::HW_IF_VELOCITY,
								       &hw_states_velocity_[i]) );
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
								       hardware_interface::HW_IF_EFFORT,
								       &hw_states_effort_[i]) );
    }
    return state_interfaces;
  }
  
  std::vector<hardware_interface::CommandInterface> GalilSystemHardwareInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
									   hardware_interface::HW_IF_POSITION,
									   &hw_commands_position_[i]) );
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
									   hardware_interface::HW_IF_VELOCITY,
									   &hw_commands_velocity_[i]) );
    }
    return command_interfaces;
  }
  
  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  
  hardware_interface::return_type GalilSystemHardwareInterface::read( const rclcpp::Time& /*time*/,
								      const rclcpp::Duration& /*period*/){
    //RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Reading...");
    GSize BUFFER_LENGTH=1024;
    GSize bytes_returned;
    char buffer[1024];
    if( GCommand(connection, "TP", buffer, BUFFER_LENGTH, &bytes_returned ) == G_NO_ERROR ){
      char * pch;
      pch = strtok(buffer,",");
      for( std::size_t i=0; i<info_.joints.size(); i++ ){
	if(pch != NULL){
	  hw_states_position_[i] = atof( pch );
	  pch = strtok (NULL, ",");
	}
      }
    }
    else{
      RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send TP command to Galil.");
      hardware_interface::return_type::ERROR;
    }
    if( GCommand(connection, "TV", buffer, BUFFER_LENGTH, &bytes_returned ) == G_NO_ERROR ){
      char * pch;
      pch = strtok(buffer,",");
      for( std::size_t i=0; i<info_.joints.size(); i++ ){
	if(pch != NULL){
	  hw_states_velocity_[i] = atof( pch );
	  pch = strtok (NULL, ",");
	}
      }
    }
    else{
      RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send TV command to Galil.");
      hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  
  hardware_interface::return_type GalilSystemHardwareInterface::write(const rclcpp::Time& /*time*/,
								      const rclcpp::Duration& /*period*/){

    char channels[]="ABCD";

    for( std::size_t i=0; i<info_.joints.size(); i++ ){
      if( !isnan(hw_commands_position_[i]) ){
	GSize BUFFER_LENGTH=1024;
	GSize bytes_returned;
	char command[1024];
	char buffer[1024];
    
	sprintf( command, "PA%c=%d", channels[i], ((int)hw_commands_position_[i]) );
	//std::cout << command << std::endl;

	int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
	if( error == G_NO_ERROR ){
	  sprintf( command, "BG%c", channels[i] );
	  //std::cout << command << std::endl;
	  
	  error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
	  if( error == G_NO_ERROR ){}
	  else{
	    RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send command: %s, error code %d", command, error);
	    //std::cout << buffer << " " << error << std::endl;
	    hardware_interface::return_type::ERROR;
	  }
	  
	}
	else{
	  RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send command: %s, error code %d", command, error);
	  //std::cout << buffer << " " << error << std::endl;
	  hardware_interface::return_type::ERROR;
	}
      }
      else{
	//RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "NaN command. Not sending.");
      }

    }
    return hardware_interface::return_type::OK;
  }
  
}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(galil_driver::GalilSystemHardwareInterface, hardware_interface::SystemInterface)
