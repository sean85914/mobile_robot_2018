void update_ratio(void)
{
	long ts = millis();
	double length_ = 0;
	double data_0_len = 0;
	while(millis() - ts <= 100){ // receive for 0.2 second
		++length_;
		if(digitalRead(IR) == 0){
			++data_0_len;
		}
	}
	ratio = data_0_len / float(length_);
  ratio_data.data = ratio;
  pub_ratio.publish(&ratio_data);
	if(ratio <= bound[door_idx] and ratio >= bound[door_idx+1]) find_door = true;
	else find_door = false;
}
