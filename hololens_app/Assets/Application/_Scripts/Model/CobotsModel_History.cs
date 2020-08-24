using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient.Messages;

namespace Cobots
{

    [System.Serializable]
    public class History
    {
        public List<HistoryEntry> history { get; private set; }
        public int depth { get; private set; }

        public History(int depth=10)
        {
            history = new List<HistoryEntry>();
            this.depth = depth;
        }

        public void Append(HistoryEntry entry)
        {
            //TODO
        }

        public HistoryEntry GetCurrentEntry()
        {
            if (history.Count > 0)
            {
                return history[history.Count - 1];
            }
            else
            {
                return null;
            }
        }

        public VersionTag GetCurrentVersion()
        {
            if (history.Count > 0)
            {
                return history[history.Count - 1].versionTag;
            }
            else
            {
                return null;
            }
        }

        public VersionTag GetPreviousVersion()
        {
            if (history.Count > 1)
            {
                return history[history.Count - 2].versionTag;
            }
            else
            {
                return null;
            }
        }
    }


    [System.Serializable]
    public class HistoryEntry
    {
        public string action { get; set; }
        public Dictionary<string,object> changes { get; set; }
        public Dictionary<string, object> snapshot { get; set; }
        public VersionTag versionTag { get; set; }

        public HistoryEntry(string action, Dictionary<string, object> changes, VersionTag verisonTag = null,
                       Dictionary<string, object> snapshot = null, string source = null)
        {
            this.action = action;
            this.changes = changes;
            this.snapshot = snapshot;

            if (versionTag != null)
            {
                this.versionTag = versionTag;
            }
            else if (source != null) 
            {
                this.versionTag = new VersionTag(source);
            }
            else
            {
                throw new System.Exception("Must either provide version tag or source for new tag");
            }
        }

        public Dictionary<string,object> ToDict()
        {
            var dct = new Dictionary<string, object>();
            dct.Add("action", action);
            dct.Add("changes", changes);
            dct.Add("snapshot", snapshot);
            dct.Add("version_tag", versionTag.ToDict());
            return dct;
        }

        public static HistoryEntry FromDict(Dictionary<string,object> dct) 
        {
            return new HistoryEntry(
                action: (string)dct["action"],
                changes: (Dictionary<string,object>)dct["changes"],
                snapshot: (Dictionary<string,object>)dct["snapshot"],
                verisonTag: VersionTag.FromDict((Dictionary<string,object>)dct["version_tag"])
            );
        }
    }


    [System.Serializable]
    public class VersionTag
    {
        public string source { get; set; }
        public float timestamp { get; set; }
        public string uuid { get; set; }

        public VersionTag(string source, float timestamp=-1, string uuid=null)
        {
            this.source = source;
            this.timestamp = (timestamp != -1) ? timestamp : System.DateTimeOffset.Now.ToUnixTimeSeconds();
            this.uuid = (uuid != null) ? uuid : GenerateUuid("version-tag");
        }

        public Dictionary<string,object> ToDict()
        {
            var dct = new Dictionary<string, object>();
            dct.Add("source", source);
            dct.Add("timestamp", timestamp);
            dct.Add("uuid", uuid);
            return dct;
        }

        public static VersionTag FromDict(Dictionary<string,object> dct)
        {
            return new VersionTag(
                source: (string)dct["source"],
                timestamp: (float)dct["timestamp"],
                uuid: (string)dct["uuid"]
            );
        }

        public Version ToRos()
        {
            var msg = new Version();
            msg.source.data = source;
            msg.timestamp = new RosSharp.RosBridgeClient.Messages.Standard.Time();
            msg.timestamp.secs = (uint)timestamp;
            msg.uuid.data = uuid;
            return msg;
        }

        public static VersionTag FromRos(Version tag)
        {
            return new VersionTag(
                source: tag.source.data,
                timestamp: tag.timestamp.secs,
                uuid: tag.uuid.data
            );
        }

        public override bool Equals(object obj)
        {
            if (obj is VersionTag)
            {
                return this.Equals((VersionTag)obj);
            }
            return false;
        }

        public bool Equals(VersionTag t)
        {
            return t.uuid == uuid;
        }

        public static bool operator ==(VersionTag lhs, VersionTag rhs)
        {
            return lhs.Equals(rhs);
        }

        public static bool operator !=(VersionTag lhs, VersionTag rhs)
        {
            return !(lhs.Equals(rhs));
        }

        public override int GetHashCode()
        {
            return uuid.GetHashCode();
        }

        public static string GenerateUuid(string type)
        {
            return type + "-ar-" + System.Guid.NewGuid().ToString();
        }
    }

}
