using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using ROS2;

namespace AWSIM
{
    public class LapTimer : MonoBehaviour
    {
        public GameObject lapTimerText;
        public string topic = "/diagnostics";
        [SerializeField, Range(1, 50)] int publishHz = 30;

        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };
        IPublisher<diagnostic_msgs.msg.DiagnosticArray> diagnosticPublisher;
        diagnostic_msgs.msg.DiagnosticArray diagnosticMsg;
        List<diagnostic_msgs.msg.KeyValue> keyValueList;

        const float timeTriggerThreshold = 5.0f;
        private float startTime;
        private float currentLapTime;
        private float bestLapTime;
        private float totalLapsTime;
        private uint totalLaps;
        private bool hasStartedLap = false;
        private float timer;

        void Start()
        {
            // Initialize base keys for LapTimer
            keyValueList = new List<diagnostic_msgs.msg.KeyValue>()
            {
                new diagnostic_msgs.msg.KeyValue()
                {
                    Key = "CURRENT_LAP",
                    Value = "0.0",
                },
                new diagnostic_msgs.msg.KeyValue()
                {
                    Key = "BEST_LAP",
                    Value = "0.0",
                },
                new diagnostic_msgs.msg.KeyValue()
                {
                    Key = "TOTAL_TIME",
                    Value = "0.0",
                },
                new diagnostic_msgs.msg.KeyValue()
                {
                    Key = "TOTAL_LAPS",
                    Value = "0",
                },
            };

            // Initialize diagnostic message
            diagnosticMsg = new diagnostic_msgs.msg.DiagnosticArray()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = "base_link",
                },
                Status = new diagnostic_msgs.msg.DiagnosticStatus[]
                {
                    new diagnostic_msgs.msg.DiagnosticStatus()
                    {
                        Level = 0,
                        Name = "LapTimer",
                        Message = "Lap time report",
                        Hardware_id = "LapTimer"
                    },
                },
            };

            // Create publisher
            var qos = qosSettings.GetQoSProfile();
            diagnosticPublisher = SimulatorROS2Node.CreatePublisher<diagnostic_msgs.msg.DiagnosticArray>(topic, qos);
        }

        void FixedUpdate()
        {
            if (hasStartedLap == false)
                return;

            // Update timer.
            timer += Time.deltaTime;

            // Matching publish to hz.
            var interval = 1.0f / publishHz;
            interval -= 0.00001f;       // Allow for accuracy errors.
            if (timer < interval)
                return;
            timer = 0;

            // Update Stamp
            var time = SimulatorROS2Node.GetCurrentRosTime();
            var diagnosticMsgHeader = diagnosticMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref diagnosticMsgHeader);
            keyValueList[0].Value = currentLapTime.ToString("F3");
            diagnosticMsg.Status[0].Values = keyValueList.ToArray();

            // Publish
            diagnosticPublisher.Publish(diagnosticMsg);
        }

        void Update()
        {
            // Update GUI text
            if (hasStartedLap)
            {
                currentLapTime = Time.time - startTime;
                lapTimerText.GetComponent<Text>().text = 
                    $"Current lap: {currentLapTime.ToString("F3")}\n"
                    + $"Best lap: {bestLapTime.ToString("F3")}\n"
                    + $"Total time: {totalLapsTime.ToString("F3")}\n"
                    + $"Total laps: {totalLaps.ToString()}";
            }
        }

        private void OnTriggerEnter(Collider other)
        {
            // Validate finish line crossing by vehicle
            if (currentLapTime > timeTriggerThreshold)
            {
                if (currentLapTime < bestLapTime || bestLapTime == 0.0)
                {
                    bestLapTime = currentLapTime;
                }
                totalLapsTime += currentLapTime;
                totalLaps++;

                // Update diagnostic
                keyValueList.Add(new diagnostic_msgs.msg.KeyValue()
                {
                    Key =  $"LAP_{totalLaps.ToString("D3")}",
                    Value = currentLapTime.ToString("F3"),
                });
                keyValueList[1].Value = bestLapTime.ToString("F3");
                keyValueList[2].Value = totalLapsTime.ToString("F3");
                keyValueList[3].Value = totalLaps.ToString();

                // Reset current lap timer
                currentLapTime = 0.0f;
            }

            // Start lap timer
            hasStartedLap = true;
            startTime = Time.time;
        }

        private void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<diagnostic_msgs.msg.DiagnosticArray>(diagnosticPublisher);
        }
    }
}