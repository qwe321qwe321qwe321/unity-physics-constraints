/******************************************************************************/
/*
  Project - Physics Constraints
            https://github.com/TheAllenChou/unity-physics-constraints
  
  Author  - Ming-Lun "Allen" Chou
  Web     - http://AllenChou.net
  Twitter - @TheAllenChou
*/
/******************************************************************************/

using CjLib;
using PhysicsConstraints;
using UnityEngine;

public class InlineRotationConstraintMain : MonoBehaviour
{
  public float Beta = 0.02f;

	public GameObject Box;

	private float mass;
	private float massInv;
	private Matrix3x3 inertia;
	private Matrix3x3 inertiaInv;
	[SerializeField]
	private Vector3 a = Vector3.zero;

	private void Start() {
		mass = 1.0f;
		massInv = 1.0f / mass;

		inertia = Inertia.SolidBox(mass, 1.0f * Vector3.one);
		inertiaInv = inertia.Inverted;
	}

	private void Update() {
		if (Box == null)
			return;

		float dt = Time.deltaTime;
		Box.transform.rotation.ToAngleAxis(out float qAngle, out Vector3 qAxis);
		Vector3 c = Mathf.Deg2Rad * qAngle * qAxis;
		c.x = 0f;
		c.y = 0f;

		Matrix3x3 effectiveMass = inertia;
		Vector3 jV = a;
		jV.x = 0f;
		jV.y = 0f;
		Vector3 lambda = effectiveMass * (-(jV + (Beta / dt) * c));

		a += inertiaInv * lambda;
		//a *= 0.9f; // temp magic cheat

		// integration
		Quaternion q = QuaternionUtil.AxisAngle(VectorUtil.NormalizeSafe(a, Vector3.forward), a.magnitude * dt);
		Box.transform.rotation = q * Box.transform.rotation;
	}
}
