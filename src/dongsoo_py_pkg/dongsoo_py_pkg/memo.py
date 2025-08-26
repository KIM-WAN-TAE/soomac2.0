from dongsoo_py_pkg.read_json import *

grip_dh = GripperDH()
grip_d = grip_dh.get_parameter_list('a')
grip_a = grip_dh.get_parameter_list('a')
grip_alpha = grip_dh.get_parameter_list('alpha')
grip_th_off = grip_dh.get_parameter_list('theta_offset')

print(grip_d)
print(grip_a)
print(grip_alpha)
print(grip_th_off)
