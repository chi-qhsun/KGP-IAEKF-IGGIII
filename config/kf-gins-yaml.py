import yaml

with open('kf-gins.yaml', 'r', encoding='utf-8') as file:
    config = yaml.safe_load(file)

# Accessing configuration parameters
imupath = config['imupath']
gnsspath = config['gnsspath']
outputpath = config['outputpath']
imudatalen = config['imudatalen']
imudatarate = config['imudatarate']
starttime = config['starttime']
endtime = config['endtime']
initpos = config['initpos']
initvel = config['initvel']
initatt = config['initatt']
initgyrbias = config['initgyrbias']
initaccbias = config['initaccbias']
initgyrscale = config['initgyrscale']
initaccscale = config['initaccscale']
initposstd = config['initposstd']
initvelstd = config['initvelstd']
initattstd = config['initattstd']
imunoise = config['imunoise']
antlever = config['antlever']

# print("IMU data filepath:", imupath)
# print("GNSS data filepath:", gnsspath)
# print("Output folder path:", outputpath)
# print("IMU data columns:", imudatalen)
# print("IMU datarate [Hz]:", imudatarate)
# print("Start time:", starttime)
# print("End time:", endtime)
# print("Initial position:", initpos)