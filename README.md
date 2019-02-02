# TSCC_Interleaved_MPC
A motion planning framework used for a larger planning horizon, exploiting reactive nature of tscc for local planning.

Change the path of the MPC origin file in newswitch.py
newswitch.py if written to call mpc and tscc
timescalingex.py calls tscc
launch_iros.m calls mpc 
matlab engine on python3 is needed to run the code 
input settings are done both in newswitch and get_input.m 
