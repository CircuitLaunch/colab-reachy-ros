# This is a message that holds data to describe the temperatures of a set of joint motors. 
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the temperatures were recorded. All the temperatures
# in one message have to be recorded at the same time.
#
# The arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] temperature