




test_array = [1.34,3.23,5423,423,536.5234,345.00]
test_array2 = [str(i) for i in test_array]
test_string = ",".join(test_array2)
running_count = 0
prev = 0
angles = [0,0,0,0,0,0]
for i in range(len(test_string)):
    if test_string[i] == ',':
        alpha_val = test_string[prev:i]
        angles[running_count] = alpha_val
        running_count += 1
        prev = i + 1

alpha_val = test_string[prev:i]
angles[running_count] = alpha_val


print(angles)





