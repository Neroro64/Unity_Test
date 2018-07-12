using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spawner : MonoBehaviour {
    public int num;
    public GameObject prefab;

    IEnumerator spawn()
    {
        for (int i = 0; i < num; i++)
        {
            GameObject cube = (GameObject)Instantiate(prefab, transform);
            cube.transform.localPosition = new Vector3();
            cube.GetComponent<Rigidbody>().AddForce(new Vector3(0, -5, 0));
            cube.transform.rotation = Quaternion.identity * Random.rotation;
            yield return new WaitForSeconds(0.001f);
        }
    }

    private void Start()
    {
        StartCoroutine(spawn());   
    }

}
