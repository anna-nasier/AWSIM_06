// Copyright 2022 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;
using AWSIM;
using AWSIM.LaserFormat;
using RGLUnityPlugin;
using ROS2;
using UnityEngine.Profiling;
using System;
using System.Linq;


namespace AWSIM
{
    /// <summary>
    /// R2FU integration for Robotec GPU Lidar Unity Plugin.
    /// </summary>
    [RequireComponent(typeof(LidarSensor))]
    public class RglLidar2dPublisher : MonoBehaviour
    {
        public string scanTopic = "lidar/scan";
        public string frameID = "world";

        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 5,
        };

        private Publisher<sensor_msgs.msg.LaserScan> scanPublisher;
        private sensor_msgs.msg.LaserScan scanSensorMsg;

        private RGLNodeSequence rglSubgraphUnity2Ros;
        private RGLNodeSequence rglSubgraphScan;

        private byte[] scanData;
        private LidarSensor lidarSensor;

        private void Start()
        {
            lidarSensor = GetComponent<LidarSensor>();
            lidarSensor.onNewData += OnNewLidarData;

            rglSubgraphUnity2Ros = new RGLNodeSequence()
                .AddNodePointsTransform("UNITY_TO_ROS", ROS2.Transformations.Unity2RosMatrix4x4());
            lidarSensor.ConnectToLidarFrame(rglSubgraphUnity2Ros);

            scanData = new byte[0];
            scanPublisher = SimulatorROS2Node.CreatePublisher<sensor_msgs.msg.LaserScan>(scanTopic, qosSettings.GetQoSProfile());
            scanSensorMsg = FormatLaserScan.GetSensorMsg();
            scanSensorMsg.SetHeaderFrame(frameID);
            scanSensorMsg.Angle_min = lidarSensor.configuration.minHAngle * Mathf.Deg2Rad;
            scanSensorMsg.Angle_max = lidarSensor.configuration.maxHAngle * Mathf.Deg2Rad;
            scanSensorMsg.Angle_increment =  (lidarSensor.configuration.maxHAngle - lidarSensor.configuration.minHAngle) * Mathf.Deg2Rad / (lidarSensor.configuration.HorizontalSteps - 1);
            scanSensorMsg.Time_increment = lidarSensor.configuration.laserArrayCycleTime;
            scanSensorMsg.Scan_time = 1.0f / lidarSensor.AutomaticCaptureHz;
            scanSensorMsg.Range_min = lidarSensor.configuration.minRange;
            scanSensorMsg.Range_max = lidarSensor.configuration.maxRange;

            rglSubgraphScan = new RGLNodeSequence()
                .AddNodePointsFormat("SCAN", FormatLaserScan.GetRGLFields());
            RGLNodeSequence.Connect(rglSubgraphUnity2Ros, rglSubgraphScan);
        }

        private void OnNewLidarData()
        {
            Profiler.BeginSample("Publish Scans");

            int hitCount = rglSubgraphScan.GetResultDataRaw(ref scanData, 8);
            PublishFormat(scanPublisher, scanSensorMsg, scanData, hitCount);

            Profiler.EndSample();
        }

        private void PublishFormat(Publisher<sensor_msgs.msg.LaserScan> publisher, sensor_msgs.msg.LaserScan msg,
            byte[] data, int hitCount)
        {
            float[] ranges = Enumerable.Repeat(float.NaN, lidarSensor.configuration.HorizontalSteps).ToArray();
            for (var i = 0; i < hitCount; i++)
            {
                int idx = lidarSensor.configuration.HorizontalSteps - 1 - (int)BitConverter.ToUInt32(data, i * (sizeof(float) + sizeof(UInt32)) + sizeof(float));
                float value = BitConverter.ToSingle(data, i * (sizeof(float) + sizeof(UInt32)));
                ranges[idx] = value;
            }
            var header = msg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref header);
            msg.Ranges = ranges;
            publisher.Publish(msg);
        }

        private void OnDisable()
        {
            if(scanPublisher != null) scanPublisher.Dispose();
        }
    }
}