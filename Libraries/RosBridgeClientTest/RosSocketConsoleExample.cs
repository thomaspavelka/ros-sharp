/*
© Siemens AG, 2017-2019
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using std_srvs = RosSharp.RosBridgeClient.MessageTypes.Std;
using rosapi = RosSharp.RosBridgeClient.MessageTypes.Rosapi;
using RosSharp.RosBridgeClient.MessageTypes.Moveit;


// commands on ROS system:
// launch before starting:
// roslaunch rosbridge_server rosbridge_websocket.launch
// rostopic echo /publication_test
// rostopic pub /subscription_test std_msgs/String "subscription test message data"

// launch after starting:
// rosservice call /service_response_test

namespace RosSharp.RosBridgeClientTest
{
    public class RosSocketConsole
    {
        static readonly string uri = "ws://172.25.23.48:9090";

        public static void Main(string[] args)
        {
            //RosSocket rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketSharpProtocol(uri));
            RosSocket rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketNetProtocol(uri));

            ServiceCheck(rosSocket);

            Console.WriteLine("Press any key to close...");
            Console.ReadKey(true);
            rosSocket.Close();
        }

        public static void ServiceCheck(RosSocket rosSocket)
        {
            // Create a PositionIKRequest object
            PositionIKRequest positionIKRequest2 = new PositionIKRequest();

            positionIKRequest2.group_name = "panda_arm";
            //positionIKRequest2.pose_stamped.header.frame_id = "panda_link0";

            positionIKRequest2.pose_stamped.pose.position.x = 0.5;
            positionIKRequest2.pose_stamped.pose.position.y = 0.0;
            positionIKRequest2.pose_stamped.pose.position.z = 0.5;

            positionIKRequest2.pose_stamped.pose.orientation.x = 0.0;
            positionIKRequest2.pose_stamped.pose.orientation.y = 0.0;
            positionIKRequest2.pose_stamped.pose.orientation.z = 0.0;
            positionIKRequest2.pose_stamped.pose.orientation.w = 1.0;

            // Create a GetPositionIKRequest object
            GetPositionIKRequest getPositionIKRequest = new GetPositionIKRequest(positionIKRequest2);

            // Call ROS service
            rosSocket.CallService<GetPositionIKRequest, GetPositionIKResponse>("/compute_ik", ServiceCallHandlerCheckIK, getPositionIKRequest);
        }

        public static void OriginalExample(RosSocket rosSocket)
        {
            // Publication:
            std_msgs.String message = new std_msgs.String
            {
                data = "publication test masdasdessage data"
            };

            string publication_id = rosSocket.Advertise<std_msgs.String>("publication_test");
            rosSocket.Publish(publication_id, message);

            // Subscription:
            string subscription_id = rosSocket.Subscribe<std_msgs.String>("/subscription_test", SubscriptionHandler);
            subscription_id = rosSocket.Subscribe<std_msgs.String>("/subscription_test", SubscriptionHandler);

            // Service Call:
            rosSocket.CallService<rosapi.GetParamRequest, rosapi.GetParamResponse>("/rosapi/get_param", ServiceCallHandler, new rosapi.GetParamRequest("/rosdistro", "default"));

            // Service Response:
            string service_id = rosSocket.AdvertiseService<std_srvs.TriggerRequest, std_srvs.TriggerResponse>("/service_response_test", ServiceResponseHandler);

            Console.WriteLine("Press any key to unsubscribe...");
            Console.ReadKey(true);
            rosSocket.Unadvertise(publication_id);
            rosSocket.Unsubscribe(subscription_id);
            rosSocket.UnadvertiseService(service_id);
        }

        private static void SubscriptionHandler(std_msgs.String message)
        {
            Console.WriteLine((message).data);
        }

        private static void ServiceCallHandler(rosapi.GetParamResponse message)
        {
            Console.WriteLine("ROS distro: " + message.value);
        }

        private static bool ServiceResponseHandler(std_srvs.TriggerRequest arguments, out std_srvs.TriggerResponse result)
        {
            result = new std_srvs.TriggerResponse(true, "service response message");
            return true;
        }

        private static void ServiceCallHandlerCheckIK(GetPositionIKResponse message)
        {
            Console.WriteLine("Position: " + message.solution.joint_state.ToString());
            Console.WriteLine("Error Code: " + message.error_code.ToString());
        }
    }
}