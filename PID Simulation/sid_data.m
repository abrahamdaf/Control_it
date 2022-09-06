duty = xlsread("Motor_Identification.xlsx","Sheet1","A1:A36");
rpm_l = xlsread("Motor_Identification.xlsx","Sheet1", "B1:B36");
rpm_r = xlsread("Motor_Identification.xlsx", "Sheet1", "C1:C36");
left_side = iddata(rpm_l,duty,1);
right_side = iddata(rpm_r,duty,1);