# The following gets the robot number from its IP address (the robots
# have their IP address set by the router---and in this case the robot
# number is the last octet of the IP address minus 100).  Once we have
# the robot number, cvss_config.json is updated to replace 
# INSERT_ROBOT_NUMBER with the actual number.
IP_ADDRESS=$(/sbin/ifconfig | grep 'inet '| grep -v '127.0.0.1' | tail -1 | cut -d' ' -f10 | awk '{ print $1}')
IP_LAST_OCTET=`echo $IP_ADDRESS | cut -d . -f 4`
let ROBOT_NUMBER=$IP_LAST_OCTET-100
echo "Robot Number: $ROBOT_NUMBER"
sed -i "s/INSERT_ROBOT_NUMBER/$ROBOT_NUMBER/g" kodama_client/cvss_config.json