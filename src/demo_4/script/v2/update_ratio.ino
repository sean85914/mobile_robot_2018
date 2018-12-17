void update_ratio(void)
{
	long ts = millis();
	int length_ = 0;
	int data_0_len = 0;
	while(millis() - ts <= 200){ // receive for 0.2 second
		++length_;
		if(digitalRead(IR) == 0){
			++data_0_len;
		}
	}
	ratio = data_0_len / float(length_);
	if(ratio <= bound[door_idx] and ratio >= bound[door_idx+1]) find_door = true;
	else find_door = false;
}
