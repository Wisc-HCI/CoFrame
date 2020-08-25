using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Cobots;

public class CollisionMeshManagerTestScript : MonoBehaviour
{
    public CollisionMeshManagerScript collisionManager;

    public string[] collisionNames = new string[4];
    public Toggle[] visibilityToggle = new Toggle[4];
    public Dropdown[] stateDropdowns = new Dropdown[4];

    private void Start()
    {
        for (int i = 0; i < 4; i++)
        {
            OnVisibleToggle(i);
            OnDropdownChange(i);
        }
    }

    public void OnVisibleToggle(int idx)
    {
        collisionManager.SetVisible(collisionNames[idx], visibilityToggle[idx].isOn);
    }

    public void OnDropdownChange(int idx)
    {
        string state = null;
        switch (stateDropdowns[idx].value)
        {
            case 0:
                state = CollisionMesh.GOOD_STATE;
                break;
            case 1:
                state = CollisionMesh.WARN_STATE;
                break;
            case 2:
                state = CollisionMesh.ERROR_STATE;
                break;
        }

        CollisionMesh p = new CollisionMesh(state: state, name: collisionNames[idx]);
        collisionManager.OnCollisionMeshUpdate(p);
    }
}
