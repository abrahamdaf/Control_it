duty = xlsread("respuesta de motores (1).xlsx", "Hoja1", "A7:A34");
rpm_l = xlsread("respuesta de motores (1).xlsx", "Hoja1", "C7:C34");
rpm_r = xlsread("respuesta de motores (1).xlsx", "Hoja1", "B7:B34");
left_side = iddata(rpm_l,duty,1);
right_side = iddata(rpm_r,duty,1);