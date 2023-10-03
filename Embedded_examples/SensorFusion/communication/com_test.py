import ast

my_dict = {'A': 1, 'B': 2, 'C': "Apple"}

my_string = str(my_dict)

print(my_string)

resonstructed_dict = ast.literal_eval(my_string)

print(f"Original: {my_dict['C']}")
print(f"New     : {resonstructed_dict['C']}")