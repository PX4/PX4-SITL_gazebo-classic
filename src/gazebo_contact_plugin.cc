#include "gazebo_contact_plugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
	// Get the parent sensor.
	this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
	
	// Make sure the parent sensor is valid.
	if (!this->parentSensor)
	{
		gzerr << "ContactPlugin requires a ContactSensor.\n";
		return;
	}
	
	// Connection to the sensor update event.
	this->updateConnection = this->parentSensor->ConnectUpdated(
		std::bind(&ContactPlugin::OnUpdate, this));
		
	// Make sure the parent sensor is active.
	this->parentSensor->SetActive(true);
}

std::string prev_string;

std::string first_leg;
std::string second_leg;
std::string third_leg;
std::string fourth_leg;

bool gate = 0;
bool leg1_status = 0;
bool leg2_status = 0;
bool leg3_status = 0;
bool leg4_status = 0;

int score = 0;

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
	// Get all the contacts.
	msgs::Contacts contacts;
	contacts = this->parentSensor->Contacts();
	
	for (unsigned int i = 0; i < contacts.contact_size(); i++) {
		std::string str(contacts.contact(i).collision2());
		
		std::string strcpy(contacts.contact(i).collision1());
		std::string leg_1("lander_0::leg_1::collision_1");
		std::string leg_2("lander_0::leg_2::collision_0");
		std::string leg_3("lander_0::leg_3::collision_0");
		std::string leg_4("lander_0::leg_4::collision_0");
		
		std::string inner_ring("landing_zone::inner_ring::collision");
		std::string middle_ring("landing_zone::middle_ring::collision");
		std::string outer_ring("landing_zone::outer_ring::collision");
			
		if (((::first_leg != str) || (::first_leg.size() == 0)) && (strcpy == leg_1)) {
			::first_leg = str;
			::leg1_status = 1;
		}
		else if (((::second_leg != str) || (::second_leg.size() == 0)) && (strcpy == leg_2)) {
			::second_leg = str;
			::leg2_status = 1;
		}
		else if (((::third_leg != str) || (::third_leg.size() == 0)) && (strcpy == leg_3)) {
			::third_leg = str;
			::leg3_status = 1;
		}
		else if (((::fourth_leg != str) || (::fourth_leg.size() == 0)) && (strcpy == leg_4)) {
			::fourth_leg = str;
			::leg4_status = 1;
		}
		else {
			if ((::prev_string != str) && ((::first_leg == ::second_leg) && (::second_leg == ::third_leg) && (::third_leg == ::fourth_leg))) {
				::prev_string = str;
				::gate = 1;
			}
			else {
			
				if ((::prev_string != middle_ring) && ((((::first_leg == outer_ring) && (::leg1_status == 0)) || ((::second_leg == outer_ring) && (::leg2_status == 0)) || ((::third_leg == outer_ring) && (::leg3_status == 0)) || ((::fourth_leg == outer_ring) && (::leg4_status == 0))) || (((::first_leg == inner_ring) && (::leg1_status == 1)) || ((::second_leg == inner_ring) && (::leg2_status == 1)) || ((::third_leg == inner_ring) && (::leg3_status == 1)) || ((::fourth_leg == inner_ring) && (::leg4_status == 1)))) && (((::first_leg == middle_ring) && (::leg1_status == 1)) || ((::second_leg == middle_ring) && (leg2_status == 1)) || ((::third_leg == middle_ring) && (leg3_status == 1)) || ((::fourth_leg == middle_ring) && (leg4_status == 1)))) {					
					::prev_string = middle_ring;
					::gate = 1;
				}
				else if ((::prev_string != outer_ring) && (((::first_leg == outer_ring) && (::leg1_status == 1)) || ((::second_leg == outer_ring) && (::leg2_status == 1)) || ((::third_leg == outer_ring) && (::leg3_status == 1)) || ((::fourth_leg == outer_ring) && (::leg4_status == 1)))) {
					::prev_string = outer_ring;
					::gate = 1;
				}
				else {
					::gate = 0;
				}
			}
		}
		
		if (::gate == 1) {
			std::cout << "Collision between[" << contacts.contact(i).collision1() << "] and [" << ::prev_string << "]\n";
			
			if (::prev_string == inner_ring) {
				score = score + 20;
			}
			else if (::prev_string == middle_ring) {
				score = score + 10;
			}
			else if (::prev_string == outer_ring) {
				score = score + 5;
			}
			else {
				score = score + 0;
			}
			
			std::cout << "Score: " << score << std::endl;
			
			::gate = 0;
				
			::leg1_status = 0;
			::leg2_status = 0;
			::leg3_status = 0;
			::leg4_status = 0;
		}
        }
}
