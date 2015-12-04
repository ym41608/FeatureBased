function [Matrix Pose] = genPoseRandom(variable_type, level, intrinsic)

    while(1)
        rx = (rand() - 0.5) * 150;
        ry = (rand() - 0.5) * 150;
        tilt_angle = acosd(cosd(rx)*cosd(ry));
        if (tilt_angle > 90)
            tilt_angle = tilt_angle - 90;
        end
        switch(variable_type)
            case('tilt_angle')
                if (tilt_angle >= level*16 & tilt_angle < (level+1)*16)
                    break;
                end
            otherwise
                if (tilt_angle < 75)
                    break;
                end
        end
    end
    rz = (rand() - 0.5) * 360;
    
    tz = (rand() - 0.5) * 4.3 + 4.85;
    
    tx_width = intrinsic(1,3) * tz / intrinsic(1,1) - 0.8;
    tx = (rand() - 0.5) * tx_width * 2;
    
    ty_width = intrinsic(2,3) * tz / (-intrinsic(2,2)) - 0.8;
    ty = (rand() - 0.5) * ty_width * 2;
    
   
	A_temp = [1 0 0; 0 cosd(-rx) -sind(-rx); 0 sind(-rx) cosd(-rx)] * ... 
	       [cosd(-ry) 0 sind(-ry); 0 1 0; -sind(-ry) 0 cosd(-ry)] * ...
		   [cosd(rz) -sind(rz) 0; sind(rz) cosd(rz) 0; 0 0 1];
	
	A_temp(:,3) = -A_temp(:,3); 
	Matrix = [A_temp(1,:) tx ; A_temp(2,:) ty ; A_temp(3,:) tz; 0 0 0 1];
    Pose = [rx ry rz tx ty tz];

end