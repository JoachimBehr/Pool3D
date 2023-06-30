// Entry psoint for non-wasm
#[cfg(not(target_arch = "wasm32"))]
#[tokio::main]
async fn main() {
    run().await;
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
enum ControlType {
    Cam,
    Fly,
}


use three_d::*;
use three_d::Window;
use three_d::egui::*;
use three_d_asset::io::*;
use rapier3d::prelude::*;
use std::f32::consts::PI;
use three_d::*;
use rapier3d::prelude::*;

pub struct XRSphere {
    pub pos: Vector3<f32>,  // Assuming pos, vel, momentum, angular_vel are all f64, change as per your need
    pub vel: Vector3<f32>,
    pub init_pos: Vector3<f32>,  // Assuming pos, vel, momentum, angular_vel are all f64, change as per your need
    pub init_vel: Vector3<f32>,
    pub radius: f32,
    pub mass: f32,
    pub color: Color,
    pub restitution: f32,
    pub us: f32,
    pub ur: f32,
    pub theta: f32,
    pub angular_vel: Vector4<f32>,
    pub net_force: Vector3<f32>,
    pub f_ext: Vector3<f32>,
    pub k: f32,
    pub p: Vector3<f32>,
    pub collision: bool,
    pub momentum: Vector3<f32>,
    pub sphere: ModelPart<PhysicalMaterial>,
    pub light: PointLight,
    pub body_handle: RigidBodyHandle,
}

impl XRSphere {
    pub fn new(
            pos: Vector3<f32>, 
            vel: Vector3<f32>, 
            init_pos: Vector3<f32>, 
            init_vel: Vector3<f32>, 
            radius: f32, mass: f32, 
            sphere: ModelPart<PhysicalMaterial>, 
            context: &three_d::Context) -> XRSphere {
                let pos: Vector3<f32> = pos;
                let vel: Vector3<f32> = vel;
                let init_pos: Vector3<f32> = pos;
                let init_vel: Vector3<f32> = vel;
                let radius: f32 = radius;
                let mass: f32 = mass;
                let color: Color = Color::WHITE;
                let restitution: f32 = 0.95;
                let us: f32 = 0.05;
                let ur: f32 = 0.02;
                let theta: f32 = 0.0;
                let angular_vel: Vector4<f32> = Vector4::new(0.0, 0.0, 0.0, 0.0);
                let net_force: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0);
                let f_ext: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0);
                let k: f32 = 500.0;
                let p: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0);
                let collision: bool = false; 
                let momentum: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0);
                let context: &three_d::Context = context;
                let light: PointLight = PointLight::new(context, 1.0, Color::WHITE, &init_pos, Attenuation::default());
                let mut sphere: ModelPart<PhysicalMaterial> = sphere;
                let body_handle: RigidBodyHandle = RigidBodyHandle::default();
                sphere.set_transformation(Mat4::from_translation(init_pos) * Mat4::from_scale(1.0));
                XRSphere {
                        pos, 
                        vel, 
                        init_pos, 
                        init_vel, 
                        radius,
                        mass, 
                        color,
                        restitution,
                        us,
                        ur,
                        theta,
                        angular_vel, 
                        net_force, 
                        f_ext,  
                        k, 
                        p, 
                        collision, 
                        momentum,  
                        sphere,
                        light,
                        body_handle,
                }
    }

    pub fn init(&mut self, rigid_body_set: &mut RigidBodySet) {
        self.pos = self.init_pos;
        self.vel = Vector3::new(0.0, 0.0, 0.0);
        rigid_body_set[self.body_handle].set_linvel(vector![0.0, 0.0, 0.0], true);
        rigid_body_set[self.body_handle].set_angvel(vector![0.0, 0.0, 0.0], true);
        rigid_body_set[self.body_handle].set_position(Isometry::new(
        vector![self.init_pos.x, self.init_pos.y, self.init_pos.z], 
            vector![0.0, 0.0, 0.0]), true);
        self.angular_vel = Vector4::new(self.vel.x / self.radius, self.vel.y/self.radius,-self.vel.z/self.radius, 0.0);
    //            assert_eq!(*rigid_body_set[self.body_handle].position(), Isometry::new(vector![self.init_pos.x, self.init_pos.y, self.init_pos.z], vector![0.0, 0.0, 0.0]));
    }

    pub fn init_pos(&self) -> Vector3<f32> {
        self.init_pos
    }

    pub fn set_light(&mut self, intensity: f32, color: Color, attenuation: Attenuation) {
        self.light.color = color;
        self.light.intensity = intensity;
        self.light.attenuation = attenuation;
        let c: Vector4<f32> = color.to_vec4() * intensity;
        self.sphere.material.emissive = Color::from_rgba_slice(&[c.x, c.y, c.z, c.w]);
    }

    pub fn collision_resolver(&mut self, dt: f32) {
        let objpos: Vector3<f32> = Vector3::new(0.0, -0.0946, 0.0);
        let objsize: Vector3<f32> = Vector3::new(2.0, 0.1, 2.0);
        let ax: Vector3<f32> = self.pos - objpos;
        let v_proj_to_vx0: Vector3<f32> = ax.project_on(vec3(1.0, 0.0, 0.0));
        let v_proj_to_vy0: Vector3<f32> = ax.project_on(vec3(0.0, 1.0, 0.0));
        let v_proj_to_vz0: Vector3<f32> = ax.project_on(vec3(0.0, 0.0, 1.0));
        let distance_x: f32 = v_proj_to_vx0.magnitude();
        let distance_y: f32 = v_proj_to_vy0.magnitude();
        let distance_z: f32 = v_proj_to_vz0.magnitude();
        let diff_x: f32 = objsize.x/2.0+self.radius;
        let diff_y: f32 = objsize.y/2.0+self.radius;
        let diff_z: f32 = objsize.z/2.0+self.radius;
        let is_collisionx: bool = distance_x <= (diff_x);
        let is_collisiony: bool = distance_y <= (diff_y);
        let is_collisionz: bool = distance_z <= (diff_z);
        let is_collision = is_collisionx && is_collisiony && is_collisionz;       
        if is_collision {
            self.collision = self.collision || is_collision;
    //            println!("COLLISION!");

            let restitution: f32 = 0.5;
            let nv: Vector3<f32> = v_proj_to_vy0.normalize();

            let jn: f32 = -self.vel.dot(nv) * (1.0 + restitution);
            let veljn_proj: Vector3<f32> = nv * jn;

            let mut friction: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0);

            if self.vel.magnitude() > 1.5 && self.collision {
                self.us = 0.04;     
    //                friction = -self.us * self.vel.magnitude() * nv;           
            } else if self.vel.magnitude() > 0.0 && self.collision {
                self.us = 0.03;     
                friction = -self.us * self.vel.magnitude() * nv;           
            }

            self.vel += veljn_proj;
            self.pos += self.vel * dt;

            if self.vel.magnitude() < 0.01 {
                self.vel = Vector3::new(0.0, 0.0, 0.0);
            }
        }
    }

    pub fn runge_kutta_step(&mut self, gravity: &Vector3<f32>, dt: f32) {
    //        const gravity: Vector3<f32> = Vector3::new(0.0, -9.81, 0.0); // Assuming Earth gravity

        let k1_pos: Vector3<f32> = dt * self.vel;
        let k1_vel: Vector3<f32> = dt * *gravity;

        let k2_pos: Vector3<f32> = dt * (self.vel + 0.5 * k1_vel);
        let k2_vel: Vector3<f32> = dt * *gravity;

        let k3_pos: Vector3<f32> = dt * (self.vel + 0.5 * k2_vel);
        let k3_vel: Vector3<f32> = dt * *gravity;

        let k4_pos: Vector3<f32> = dt * (self.vel + k3_vel);
        let k4_vel: Vector3<f32> = dt * *gravity;

        self.pos += (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos) / 6.0;
        self.vel += (k1_vel + 2.0 * k2_vel + 2.0 * k3_vel + k4_vel) / 6.0;
    }

    pub fn update(&mut self, rigid_body_set: &mut RigidBodySet) {
    //        let transformed_point = rigid_body_set[self.body_handle].position().rotation.as_vector().data.0.to_vec();
    //        let x = transformed_point[0];
    //        let rotation = Vector3::new(x[0],  x[1], x[2]); 
    //        self.sphere.set_transformation(Mat4::from_translation(self.pos) * Mat4::from_translation(rotation) *  Mat4::from_scale(1.0));
        self.pos = Vector3::new(rigid_body_set[self.body_handle].translation().x, rigid_body_set[self.body_handle].translation().y, rigid_body_set[self.body_handle].translation().z); 
    //        let angvel = 2.0*rigid_body_set[self.body_handle].angvel();
        let eulers = rigid_body_set[self.body_handle].rotation().euler_angles();
    //        println!("eulers:{:?} \n", eulers);
        let rad0 = radians(eulers.0);
        let rad1 = radians(eulers.1);
        let rad2 = radians(eulers.2);
    //        println!("rads:{:?} {:?} {:?} \n", rad0, rad1, rad2);

        let rotxx = Mat4::from_angle_z(radians(eulers.2));
        let rotyy = Mat4::from_angle_y(radians(eulers.1));
        let rotzz = Mat4::from_angle_x(radians(eulers.0));
        let rot = rotxx*rotyy*rotzz;
    //        println!("rotxx:{:?} rotyy:{:?} rotzz:{:?}\n", rotxx, rotyy, rotzz);
        self.sphere.set_transformation(Mat4::from_translation(self.pos) * rot * Mat4::from_scale(1.0));
        
    }
}

fn init_queue(camera: &mut Camera) -> Mat4 {
    let target: Vector3<f32> = Vector3::new(-0.63, 0.03, 0.01);
    let position = *camera.position();
    let up =  *camera.up();
    camera.set_view(position, target, up);

    let pushdirection = camera.view_direction().normalize();
    let mut rotx = Mat4::from_angle_y(radians(0.0));
    let mut roty = Mat4::from_angle_y(radians(0.0));
    let mut rotz = Mat4::from_angle_y(radians(0.0));
    let mut rotation = rotx*roty*rotz;
    if pushdirection.x.signum() >= 0.0 {
        rotx = Mat4::from_angle_y(radians(0.0));
        roty = Mat4::from_angle_y(radians(pushdirection.y*PI));
        rotz = Mat4::from_angle_y(radians(-PI/2.0*pushdirection.z));
        rotation = rotx*rotz;
    } else {
        rotx = Mat4::from_angle_y(radians(PI));
        roty = Mat4::from_angle_y(radians(pushdirection.y*PI));
        rotz = Mat4::from_angle_y(radians(PI/2.0*pushdirection.z));
        rotation = rotx*rotz;
    }
    //                        rotation = rotxy*rotzy;
    let trans = Mat4::from_translation(target) * rotation * Mat4::from_scale(1.0);

    trans
}

pub async fn run() {

    let mut rigid_body_set: RigidBodySet = RigidBodySet::new();
    let mut collider_set: ColliderSet = ColliderSet::new();
    let init_pushpoint: Point<Real> = Point::new(-0.65, 0.07, 0.05);
    let mut pushpoint: Point<Real> = init_pushpoint.clone();
    let mut push_impulse = 1.23;
    let mut pushang = Vector::new(0.0, 0.0, 0.0);
    let mut change = false;

    let table_height = 0.004;
    let table_offset = -0.06;
    let table_pos: Vector3<f32> = Vector3::new(0.0, table_height+table_offset, 0.0); 
    let plane_pos: Vector3<f32> = Vector3::new(0.0, table_height+table_offset, 0.0); 
    let bande_pos: Vector3<f32> = Vector3::new(0.0, table_offset, 0.0); 
    let basket_pos: Vector3<f32> = Vector3::new(0.0,table_offset,0.0); 

    // GUI variables
    let mut control_type = ControlType::Cam;
    let mut color = [1.0; 4];
    let mut z_offset = 0.002;     
    let mut shadows_enabled = true;
    let mut control_type = ControlType::Cam;
    let mut lighting_model = LightingModel::Blinn;

    // Create a window (a canvas on web)
    let window = Window::new(WindowSettings {
        title: "Pool3D".to_string(),
//        max_size: Some((2400, 1080)),
        ..Default::default()
    })
    .unwrap();

    // Get the graphics context from the window
    let context = window.gl();

    let mut camera: Camera = Camera::new_perspective(
        window.viewport(),
        vec3(-3.0, 0.6, 0.0),
        vec3(0.0, 1.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(45.0),
        0.1,
        5000.0,
    );

    // GUI
    let mut gui = three_d::GUI::new(&context);

    // MODELS
    // Pool Hall
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/PoolHall3.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/PoolHall3.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut model: CpuModel = loaded.deserialize("PoolHall3").unwrap();
    model
        .geometries
        .iter_mut()
        .for_each(|m: &mut three_d_asset::Primitive| {
            m.compute_normals();
            m.compute_tangents();
            m.compute_aabb();
        });
    let pool_hall = Model::<PhysicalMaterial>::new(&context, &model).unwrap();

    // Table
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/PoolTable.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/PoolTable.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut model: CpuModel = loaded.deserialize("PoolTable").unwrap();
    model
        .geometries
        .iter_mut()
        .for_each(|m: &mut three_d_asset::Primitive| {
            m.transformation = Mat4::from_translation(table_pos) * Mat4::from_scale(1.0);
            m.compute_normals();
            m.compute_tangents();
            m.compute_aabb();
        });
    let table = Model::<PhysicalMaterial>::new(&context, &model).unwrap();

    /////////////////////////////////////////////
    // Queue
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Queue_3.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Queue_3.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut model: CpuModel = loaded.deserialize("Queue_3").unwrap();
    model
        .geometries
        .iter_mut()
        .for_each(|m: &mut three_d_asset::Primitive| {
            m.compute_normals();
            m.compute_tangents();
            m.compute_aabb();
        });
    let mut queue0 = Model::<PhysicalMaterial>::new(&context, &model)
    .unwrap()
    .remove(0);

    let mut queue1 = Model::<PhysicalMaterial>::new(&context, &model)
    .unwrap()
    .remove(1);

    let mut queue2 = Model::<PhysicalMaterial>::new(&context, &model)
    .unwrap()
    .remove(2);



// MESHES /////////////////////////////////////////////////////////////////////////////////////////////
    // Pickmesh
    let mut sphere = CpuMesh::sphere(8);
    sphere.transform(&Mat4::from_scale(0.005)).unwrap();
    let mut pick_mesh = Gm::new(
        three_d::renderer::geometry::Mesh::new(&context, &sphere),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                albedo: Color::BLUE,
                ..Default::default()
            },
        ),
    );

    // Table plate
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Pooltable35.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Pooltable35.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut plane_mesh: CpuMesh = loaded.deserialize("Pooltable35").unwrap();
    let newpos = Mat4::from_translation(plane_pos) * Mat4::from_scale(1.0);
    let _result = plane_mesh.transform(&newpos).unwrap();
    let mut vertices: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in plane_mesh.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices.push(pos);
    }
    let mut indices: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(plane_mesh.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = plane_mesh.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices.push(pos);
    }

    // Bande 1
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Bande1.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Bande1.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_b1: CpuMesh = loaded.deserialize("Bande1").unwrap();
    let newpos = Mat4::from_translation(bande_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_b1.transform(&newpos).unwrap();

    // Bande 2
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Bande2.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Bande2.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_b2: CpuMesh = loaded.deserialize("Bande2").unwrap();
    let newpos = Mat4::from_translation(bande_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_b2.transform(&newpos).unwrap();

    // Bande 3
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Bande3.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Bande3.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_b3: CpuMesh = loaded.deserialize("Bande3").unwrap();
    let newpos = Mat4::from_translation(bande_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_b3.transform(&newpos).unwrap();

    // Bande 4
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Bande4.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Bande4.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_b4: CpuMesh = loaded.deserialize("Bande4").unwrap();
    let newpos = Mat4::from_translation(bande_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_b4.transform(&newpos).unwrap();

    // Bande 5
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Bande5.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Bande5.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_b5: CpuMesh = loaded.deserialize("Bande5").unwrap();
    let newpos = Mat4::from_translation(bande_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_b5.transform(&newpos).unwrap();

    // Bande 6
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Bande6.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Bande6.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_b6: CpuMesh = loaded.deserialize("Bande6").unwrap();
    let newpos = Mat4::from_translation(bande_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_b6.transform(&newpos).unwrap();

    // Basket 1
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Basket_1.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Basket_1.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_bas1: CpuMesh = loaded.deserialize("Basket_1").unwrap();
    let newpos = Mat4::from_translation(basket_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_bas1.transform(&newpos).unwrap();

    // Basket 2
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Basket_2.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Basket_2.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_bas2: CpuMesh = loaded.deserialize("Basket_2").unwrap();
    let newpos = Mat4::from_translation(basket_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_bas2.transform(&newpos).unwrap();

    // Basket 3
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Basket_3.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Basket_3.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_bas3: CpuMesh = loaded.deserialize("Basket_3").unwrap();
    let newpos = Mat4::from_translation(basket_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_bas3.transform(&newpos).unwrap();

    // Basket 4
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Basket_4.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Basket_4.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_bas4: CpuMesh = loaded.deserialize("Basket_4").unwrap();
    let newpos = Mat4::from_translation(basket_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_bas4.transform(&newpos).unwrap();

    // Basket 5
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Basket_5.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Basket_5.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_bas5: CpuMesh = loaded.deserialize("Basket_5").unwrap();
    let newpos = Mat4::from_translation(basket_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_bas5.transform(&newpos).unwrap();

    // Basket 6
    let mut loaded = if let Ok(loaded) =
        load_async(&["assets/Pool/Basket_6.glb"]).await
    {
        loaded
    } else {
        load_async(&["assets/Pool/Basket_6.gltf"])
            .await
            .expect("failed to download the necessary assets, to enable running this example offline, place the relevant assets in a folder called 'assets' next to the three-d source")
    };
    let mut obj_mesh_bas6: CpuMesh = loaded.deserialize("Basket_6").unwrap();
    let newpos = Mat4::from_translation(basket_pos) * Mat4::from_scale(1.0);
    let _result = obj_mesh_bas6.transform(&newpos).unwrap();



// COLLIDERS /////////////////////////////////////////////////////////////////////////////////////////////
    // Ground
    let ground_size = 50.0;
    let ground_height = 0.1;
    let ground_zero = -0.837-ground_height;
    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, ground_zero, 0.0]);
    let ground_handle = rigid_body_set.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    collider_set.insert_with_parent(collider, ground_handle, &mut rigid_body_set);

    // Table plate
    let plane_pos: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0); 
    let rigid_body: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![plane_pos.x, plane_pos.y, plane_pos.z])
        .linear_damping(0.4)
        .angular_damping(0.25)
        .ccd_enabled(true)
        .can_sleep(false)
        .build();
    let body_handle: RigidBodyHandle = rigid_body_set.insert(rigid_body);
    let collider = 
        ColliderBuilder::trimesh(vertices, indices)
        .restitution(0.3)
        .friction(0.5);
    collider_set.insert_with_parent(collider, body_handle, &mut rigid_body_set);

    // Bande 1
    let bande_pos: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0); 
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_b1.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_b1.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_b1.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
        }
    let rigid_body_bande_1: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![bande_pos.x, bande_pos.y, bande_pos.z])
        .linear_damping(0.2)
        .angular_damping(0.15)
        .ccd_enabled(true)
        .can_sleep(false)
        .build();
    let body_handle_bande_1: RigidBodyHandle = rigid_body_set.insert(rigid_body_bande_1);
    let collider_bande_1 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(2.0)
            .restitution(0.3)
            .friction(0.5);
    collider_set.insert_with_parent(collider_bande_1, body_handle_bande_1, &mut rigid_body_set);

    // Bande 2
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_b2.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_b2.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_b2.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
        }
    let rigid_body_bande_2: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![bande_pos.x, bande_pos.y, bande_pos.z])
        .linear_damping(0.2)
        .angular_damping(0.15)
        .ccd_enabled(true)
        .can_sleep(false)
        .build();
    let body_handle_bande_2: RigidBodyHandle = rigid_body_set.insert(rigid_body_bande_2);
    let collider_bande_2 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(2.0)
            .restitution(0.3)
            .friction(0.5);
    collider_set.insert_with_parent(collider_bande_2, body_handle_bande_2, &mut rigid_body_set);

    // Bande 3
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_b3.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_b3.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_b3.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
        }
    let rigid_body_bande_3: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![bande_pos.x, bande_pos.y, bande_pos.z])
        .linear_damping(0.2)
        .angular_damping(0.15)
        .ccd_enabled(true)
        .can_sleep(false)
        .build();
    let body_handle_bande_3: RigidBodyHandle = rigid_body_set.insert(rigid_body_bande_3);
    let collider_bande_3 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(2.0)
            .restitution(0.3)
            .friction(0.5);
    collider_set.insert_with_parent(collider_bande_3, body_handle_bande_3, &mut rigid_body_set);

    // Bande 4
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_b4.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_b4.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_b4.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
        }
    let rigid_body_bande_4: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![bande_pos.x, bande_pos.y, bande_pos.z])
        .linear_damping(0.2)
        .angular_damping(0.15)
        .ccd_enabled(true)
        .can_sleep(false)
        .build();
    let body_handle_bande_4: RigidBodyHandle = rigid_body_set.insert(rigid_body_bande_4);
    let collider_bande_4 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(2.0)
            .restitution(0.3)
            .friction(0.5);
    collider_set.insert_with_parent(collider_bande_4, body_handle_bande_4, &mut rigid_body_set);

    // Bande 5
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_b5.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_b5.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_b5.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
        }
    let rigid_body_bande_5: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![bande_pos.x, bande_pos.y, bande_pos.z])
        .linear_damping(0.2)
        .angular_damping(0.15)
        .ccd_enabled(true)
        .can_sleep(false)
        .build();
    let body_handle_bande_5: RigidBodyHandle = rigid_body_set.insert(rigid_body_bande_5);
    let collider_bande_5 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(2.0)
            .restitution(0.3)
            .friction(0.5);
    collider_set.insert_with_parent(collider_bande_5, body_handle_bande_5, &mut rigid_body_set);

    // Bande 6
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_b6.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_b6.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_b6.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
        }
    let rigid_body_bande_6: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![bande_pos.x, bande_pos.y, bande_pos.z])
        .linear_damping(0.2)
        .angular_damping(0.15)
        .ccd_enabled(true)
        .can_sleep(false)
        .build();
    let body_handle_bande_6: RigidBodyHandle = rigid_body_set.insert(rigid_body_bande_6);
    let collider_bande_6 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(2.0)
            .restitution(0.3)
            .friction(0.5);
    collider_set.insert_with_parent(collider_bande_6, body_handle_bande_6, &mut rigid_body_set);

    // Basket 1
    let basket_pos: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0); 
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_bas1.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_bas1.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_bas1.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
    }
    let rigid_body_bas1: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![basket_pos.x, basket_pos.y, basket_pos.z])
        .linear_damping(0.9)
        .angular_damping(0.75)
        .ccd_enabled(false)
        .can_sleep(true)
        .build();
    let body_handle_bas1: RigidBodyHandle = rigid_body_set.insert(rigid_body_bas1);
    let collider_bas1 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(1.0)
            .restitution(0.1)
            .friction(0.7);
    collider_set.insert_with_parent(collider_bas1, body_handle_bas1, &mut rigid_body_set);

    // Basket 2
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_bas2.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_bas2.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_bas2.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
    }
    let rigid_body_bas2: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![basket_pos.x, basket_pos.y, basket_pos.z])
        .linear_damping(0.9)
        .angular_damping(0.75)
        .ccd_enabled(false)
        .can_sleep(true)
        .build();
    let body_handle_bas2: RigidBodyHandle = rigid_body_set.insert(rigid_body_bas2);
    let collider_bas2 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(1.0)
            .restitution(0.1)
            .friction(0.7);
    collider_set.insert_with_parent(collider_bas2, body_handle_bas2, &mut rigid_body_set);

    // Basket 3
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_bas3.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_bas3.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_bas3.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
    }
    let rigid_body_bas3: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![basket_pos.x, basket_pos.y, basket_pos.z])
        .linear_damping(0.9)
        .angular_damping(0.75)
        .ccd_enabled(false)
        .can_sleep(true)
        .build();
    let body_handle_bas3: RigidBodyHandle = rigid_body_set.insert(rigid_body_bas3);
    let collider_bas3 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(1.0)
            .restitution(0.1)
            .friction(0.7);
    collider_set.insert_with_parent(collider_bas3, body_handle_bas3, &mut rigid_body_set);

    // Basket 4
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_bas4.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_bas4.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_bas4.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
    }
    let rigid_body_bas4: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![basket_pos.x, basket_pos.y, basket_pos.z])
        .linear_damping(0.9)
        .angular_damping(0.75)
        .ccd_enabled(false)
        .can_sleep(true)
        .build();
    let body_handle_bas4: RigidBodyHandle = rigid_body_set.insert(rigid_body_bas4);
    let collider_bas4 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(1.0)
            .restitution(0.1)
            .friction(0.7);
    collider_set.insert_with_parent(collider_bas4, body_handle_bas4, &mut rigid_body_set);

    // Basket 5
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_bas5.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_bas5.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_bas5.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
    }
    let rigid_body_bas5: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![basket_pos.x, basket_pos.y, basket_pos.z])
        .linear_damping(0.9)
        .angular_damping(0.75)
        .ccd_enabled(false)
        .can_sleep(true)
        .build();
    let body_handle_bas5: RigidBodyHandle = rigid_body_set.insert(rigid_body_bas5);
    let collider_bas5 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(1.0)
            .restitution(0.1)
            .friction(0.7);
    collider_set.insert_with_parent(collider_bas5, body_handle_bas5, &mut rigid_body_set);

    // Basket 6
    let mut vertices1: Vec<Point<Real>> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    for p in obj_mesh_bas6.positions.to_f32().iter_mut() {
        let pos: Point<Real> = Point::new(p[0], p[1], p[2]);
        vertices1.push(pos);
    }
    let mut indices1: Vec<[u32; 3]> = Vec::new(); // A vector to contain VertexFormat::Float32x3
    let ind = three_d_asset::Indices::U32(obj_mesh_bas6.indices.to_u32().unwrap());
    let vec = ind.into_u32().unwrap();
    let x: usize = obj_mesh_bas6.indices.len().unwrap();
    for f in 0..x / 3 {
        let i1 = vec[3 * f] as u32;
        let i2 = vec[3 * f + 1] as u32;
        let i3 = vec[3 * f + 2] as u32;
        let pos = [i1, i2, i3];
        indices1.push(pos);
    }
    let rigid_body_bas6: RigidBody = RigidBodyBuilder::fixed()
        .translation(vector![basket_pos.x, basket_pos.y, basket_pos.z])
        .linear_damping(0.9)
        .angular_damping(0.75)
        .ccd_enabled(false)
        .can_sleep(true)
        .build();
    let body_handle_bas6: RigidBodyHandle = rigid_body_set.insert(rigid_body_bas6);
    let collider_bas6 = 
        ColliderBuilder::trimesh(vertices1, indices1)
            .density(1.0)
            .restitution(0.1)
            .friction(0.7);
    collider_set.insert_with_parent(collider_bas6, body_handle_bas6, &mut rigid_body_set);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    let mut cpu_model: CpuModel =
    three_d_asset::io::load_async(&["assets/Pool/PoolBalls32.glb"])
    .await
    .unwrap()
    .deserialize("PoolBalls32")
    .unwrap();

    cpu_model
    .geometries
    .iter_mut()
    .for_each(|m: &mut three_d_asset::Primitive| {
        m.compute_normals();
        m.compute_tangents();
        m.compute_aabb();
    });

    // Ball Cue
    let mut ball_cue:XRSphere = XRSphere::new(
    //        Vector3::new(-0.4, 0.5, 0.0), 
    Vector3::new(-0.6, table_offset+0.1, 0.0), 
    //        Vector3::new(0.0, 0.1, 0.0), 
    Vector3::new(0.0, 0.0, 0.0),
    //        Vector3::new(-0.4, 0.2, 0.0), 
    Vector3::new(-0.6, table_offset+0.1, 0.0), 
    Vector3::new(0.0, 0.0, 0.0),
    0.0286, 
    0.17, 
    Model::<PhysicalMaterial>::new(&context, &cpu_model)
        .unwrap()
        .remove(0), 
        &context);
    /* Create the bounding ball. */
    let pos = ball_cue.init_pos();
    let rigid_body: RigidBody = RigidBodyBuilder::dynamic()
    //        .translation(vector![-0.1, 0.2, 0.0])
    .translation(vector![ball_cue.init_pos.x, ball_cue.init_pos.y, ball_cue.init_pos.z])
    .additional_mass(0.170)
    .linear_damping(0.4)
    .angular_damping(0.25)
    .ccd_enabled(true)
    .can_sleep(false)
    .build();
    let body_handle: RigidBodyHandle = rigid_body_set.insert(rigid_body);
    let collider = 
    ColliderBuilder::ball(0.0286)
    .restitution(0.95)
    .friction(0.3)
    .build();
    collider_set.insert_with_parent(collider, body_handle, &mut rigid_body_set);
    ball_cue.body_handle = body_handle;    

    const LAST_BALL:usize = 15;
    let mut balls: Vec<XRSphere> = Vec::new();
    for idx in 1..=LAST_BALL {
        let ball = XRSphere::new(
        Vector3::new(0.0, table_offset+0.4, 0.0), 
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, table_offset+0.4, 0.0), 
        Vector3::new(0.0, 0.0, 0.0),
        0.0286, 
        0.17, 
        Model::<PhysicalMaterial>::new(&context, &cpu_model)
            .unwrap()
            .remove(idx), 
            &context);
    balls.push(ball);
    }

    balls[0].init_pos = Vector3::new(0.4, table_offset+0.1, 0.0);
    balls[1].init_pos = Vector3::new(0.452751, table_offset+0.1, 0.0325);
    balls[2].init_pos = Vector3::new(0.452751, table_offset+0.1, -0.0325);
    balls[3].init_pos = Vector3::new(0.508521, table_offset+0.1, 0.066631);
    balls[4].init_pos = Vector3::new(0.508521, table_offset+0.1, -0.066631);
    balls[5].init_pos = Vector3::new(0.508521, table_offset+0.1, 0.0);
    balls[6].init_pos = Vector3::new(0.564379, table_offset+0.1, 0.0333);
    balls[7].init_pos = Vector3::new(0.564379, table_offset+0.1, -0.0333);
    balls[8].init_pos = Vector3::new(0.564379, table_offset+0.1, -0.1);
    balls[9].init_pos = Vector3::new(0.564379, table_offset+0.1, 0.1);
    balls[10].init_pos = Vector3::new(0.610879 , table_offset+0.1, 0.125);
    balls[11].init_pos = Vector3::new(0.610879 , table_offset+0.1, 0.0625);
    balls[12].init_pos = Vector3::new(0.610879 , table_offset+0.1, 0.0);
    balls[13].init_pos = Vector3::new(0.610879 , table_offset+0.1, -0.0625);
    balls[14].init_pos = Vector3::new(0.610879 , table_offset+0.1, -0.125);
    balls[0].pos = balls[0].init_pos;
    balls[1].pos = balls[1].init_pos;
    balls[2].pos = balls[2].init_pos;
    balls[3].pos = balls[3].init_pos;
    balls[4].pos = balls[4].init_pos;
    balls[5].pos = balls[5].init_pos;
    balls[6].pos = balls[6].init_pos;
    balls[7].pos = balls[7].init_pos;
    balls[8].pos = balls[8].init_pos;
    balls[9].pos = balls[9].init_pos;
    balls[10].pos = balls[10].init_pos;
    balls[11].pos = balls[11].init_pos;
    balls[12].pos = balls[12].init_pos;
    balls[13].pos = balls[13].init_pos;
    balls[14].pos = balls[14].init_pos;

    for idx in 0..LAST_BALL {
    let rigid_body: RigidBody = RigidBodyBuilder::dynamic()
        .translation(vector![balls[idx].init_pos.x, balls[idx].init_pos.y, balls[idx].init_pos.z])
        .additional_mass(0.170)
        .linear_damping(0.4)
        .angular_damping(0.25)
        .ccd_enabled(true)
        .can_sleep(false)
        .build();
    let body_handle: RigidBodyHandle = rigid_body_set.insert(rigid_body);
    let collider = 
        ColliderBuilder::ball(0.0286)
        .restitution(0.95)
        .friction(0.3)
        .build();
    collider_set.insert_with_parent(collider, body_handle, &mut rigid_body_set);
    balls[idx].body_handle = body_handle;    
    }


    // Light Box
/*    
    let mut aabb = AxisAlignedBoundingBox::EMPTY;
    for m in pool_hall.iter() {
        aabb.expand_with_aabb(&m.aabb());
    }

    let size = aabb.size();
    let min = aabb.min() + vec3(size.x * 0.1, size.y * 0.1, size.z * 0.4);
    let max = aabb.max() - vec3(size.x * 0.1, size.y * 0.3, size.z * 0.4);
    let light_box = AxisAlignedBoundingBox::new_with_positions(&[min, max]);
 */

    // Lights
    let mut ambient = 
        AmbientLight::new(&context, 0.2, Color::WHITE);
    let directional = 
        DirectionalLight::new(&context, 2.0, Color::WHITE, &vec3(-1.0, -1.0, -1.0));
    let mut directional0 = 
        DirectionalLight::new(&context, 1.0, Color::RED, &vec3(0.0, -1.0, 0.0));
    let mut directional1 =
        DirectionalLight::new(&context, 1.0, Color::GREEN, &vec3(0.0, -1.0, 0.0));
    let mut spot0 = 
        SpotLight::new(
            &context,
            2.0,
            Color::BLUE,
            &vec3(0.0, 0.0, 0.0),
            &vec3(0.0, -1.0, 0.0),
            degrees(25.0),
            Attenuation {
                constant: 0.1,
                linear: 0.001,
                quadratic: 0.0001,
            },
        );
    let mut point0 = 
        PointLight::new(
            &context,
            1.0,
            Color::GREEN,
            &vec3(0.0, 0.0, 0.0),
            Attenuation {
                constant: 0.5,
                linear: 0.05,
                quadratic: 0.005,
            },
        );
    let mut point1 = 
        PointLight::new(
            &context,
            1.0,
            Color::RED,
            &vec3(0.0, 0.0, 0.0),
            Attenuation {
                constant: 0.5,
                linear: 0.05,
                quadratic: 0.005,
            },
        );

    directional0.generate_shadow_map(1024, &balls[0].sphere);
    directional1.generate_shadow_map(1024, &balls[0].sphere);
    spot0.generate_shadow_map(1024, &balls[0].sphere);


    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, -9.81, 0.0];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    let mut center_pos = rigid_body_set[ball_cue.body_handle].center_of_mass().clone();
    let mut sl_xrot = 0.0;
    let mut sl_yrot = 0.0;
    let mut sl_zrot = 0.0;
    let mut rotx = Mat4::from_angle_y(radians(sl_xrot));
    let mut roty = Mat4::from_angle_y(radians(sl_yrot));
    let mut rotz = Mat4::from_angle_y(radians(sl_zrot));
    let mut rotation = rotx*roty*rotz;

    let trans: Matrix4<f32> = init_queue(&mut camera);
    queue0.set_transformation(trans);
    queue1.set_transformation(trans);
    queue2.set_transformation(trans);
    pick_mesh.set_transformation(Mat4::from_translation(Vector3::new(pushpoint.x, pushpoint.y, pushpoint.z)));

    window.render_loop(move |mut frame_input| {
        let mut panel_width = 0.0;
        gui.update(
            &mut frame_input.events,
            frame_input.accumulated_time,
            frame_input.viewport,
            frame_input.device_pixel_ratio,
            |gui_context| {
                SidePanel::left("side_panel").show(gui_context, |ui| {
                    ui.heading("Control Panel");

                    Grid::new("g1").show(ui, |ui| {
                        ui.label("");
                        if ui.add(Button::new("Restart").min_size(three_d::egui::Vec2::new(100.0, 40.0))).clicked() {       
                            ball_cue.init(&mut rigid_body_set); 
                            for i in 0..LAST_BALL {
                                balls[i].init(&mut rigid_body_set)
                            }
                            pushpoint = init_pushpoint;
                            let trans: Matrix4<f32> = init_queue(&mut camera);
                            queue0.set_transformation(trans);
                            queue1.set_transformation(trans);
                            queue2.set_transformation(trans);
                            pick_mesh.set_transformation(Mat4::from_translation(Vector3::new(pushpoint.x, pushpoint.y, pushpoint.z)));
 
                            change = false;
                        }

                        ui.end_row();  
                    });   

                    ui.separator();

                    Grid::new("g2").show(ui, |ui| {
                        ui.label("");
                        if ui.add(Button::new("O").min_size(three_d::egui::Vec2::new(100.0, 100.0))).clicked() {
                            center_pos = rigid_body_set[ball_cue.body_handle].center_of_mass().clone();
                            let pushdirection = camera.view_direction().normalize();
                            pushang = rapier3d::math::Vector::new(pushdirection.x, pushdirection.y, pushdirection.z) * push_impulse;
                            rigid_body_set[ball_cue.body_handle].apply_impulse_at_point(pushang, pushpoint, true);

                            change = false;
                        }
                        ui.end_row();  
                    });                    

                    ui.separator();

                    Grid::new("g3").show(ui, |ui| {
                        ui.style_mut().spacing.item_spacing = three_d::egui::Vec2::new(20.0, 10.0);
                        ui.label("                 ");
                        ui.add(
                            Slider::new(&mut push_impulse, 0.001..=2.0)
                                .vertical()
                                .step_by(0.001)
                                .text("FORCE"),
                        );
                        ui.label("                 ");
                        ui.end_row();  
                    });                    

                    ui.separator();
                    
                    ui.label("Camera type");
                    ui.radio_value(&mut control_type, ControlType::Cam, "Central");
                    ui.radio_value(&mut control_type, ControlType::Fly, "Flying");

                    ui.separator();

                    ui.color_edit_button_rgba_unmultiplied(&mut color);

                    ui.separator();

                    let s = format!("{:?}", pushpoint);
                    ui.label(s);
                    let s = format!("{:?}", pushang);
                    ui.label(s);
                    let s = format!("{:?} {:?} {:?}", camera.position(), camera.target(), camera.up());
                    ui.label(s);

                    ui.allocate_space(ui.available_size());
                });
                panel_width = gui_context.used_rect().width();
            },
        );

        let viewport = Viewport {
            x: (panel_width * frame_input.device_pixel_ratio as f32) as i32,
            y: 0,
            width: frame_input.viewport.width
                - (panel_width * frame_input.device_pixel_ratio as f32) as u32,
            height: frame_input.viewport.height,
        };
        camera.set_viewport(viewport);

        let mut control1 = FlyControl::new(0.01);
        let mut control2 = CameraControl::new();
        control2.left_drag_horizontal = CameraAction::OrbitLeft { target: *camera.target(), speed: 0.01 };
        control2.left_drag_vertical = CameraAction::OrbitUp { target: *camera.target(), speed: 0.01 };
        control2.right_drag_horizontal = CameraAction::OrbitLeft { target: *camera.target(), speed: 0.001 };
        control2.right_drag_vertical = CameraAction::OrbitUp { target: *camera.target(), speed: 0.001 };
        control2.scroll_vertical =CameraAction::Zoom { target: *camera.target(), speed: 0.01, min: 0.1, max: 20.0 };

        if control_type.clone() == ControlType::Fly {
            control1.handle_events(&mut camera, &mut frame_input.events);
        } else {
            control2.handle_events(&mut camera, &mut frame_input.events);
        }

        for event in frame_input.events.iter() {
            
            if let three_d::renderer::control::Event::MousePress {
                button, position, modifiers, ..
            } = event
            {
                if modifiers.shift && control_type == ControlType::Fly {
                    control1 = FlyControl::new(0.001);
                } else if modifiers.shift && control_type == ControlType::Cam {
                    control2.scroll_vertical = CameraAction::Zoom { target: *camera.target(), speed: 0.001, min: 0.1, max: 20.0 };
                }
                if *button == MouseButton::Left {
//                    let pos:(f32, f32) = (position.0 as f32, position.1 as f32);
                    if let Some(pick) = pick(&context, &camera, position, &ball_cue.sphere) {
                        let pickpoint: Matrix4<f32> = Mat4::from_translation(pick);
                        pushpoint = Point::new(pickpoint.w.x, pickpoint.w.y, pickpoint.w.z);
                        let target: Vector3<f32> = Vector3::new(pushpoint.x, pushpoint.y, pushpoint.z);
                        let position = *camera.position();
                        let up =  *camera.up();
                        camera.set_view(position, target, up);

                        let queue_tip = Vector3::new(pushpoint.x, pushpoint.y, pushpoint.z);
                        let queue_center = queue0.aabb().center();
                        let pushdirection = camera.view_direction().normalize();
                        if pushdirection.x.signum() >= 0.0 {
                            rotx = Mat4::from_angle_y(radians(0.0));
                            roty = Mat4::from_angle_y(radians(pushdirection.y*PI));
                            rotz = Mat4::from_angle_y(radians(-PI/2.0*pushdirection.z));
                            rotation = rotx*rotz;
                        } else {
                            rotx = Mat4::from_angle_y(radians(PI));
                            roty = Mat4::from_angle_y(radians(pushdirection.y*PI));
                            rotz = Mat4::from_angle_y(radians(PI/2.0*pushdirection.z));
                            rotation = rotx*rotz;
                        }
//                        rotation = rotxy*rotzy;
                        let trans = pickpoint * rotation * Mat4::from_scale(1.0);
                        queue0.set_transformation(trans);
                        queue1.set_transformation(trans);
                        queue2.set_transformation(trans);

                        pick_mesh.set_transformation(pickpoint);

                        change = true;
                    }
                }
            }
        }
//        change |= control.handle_events(&mut camera, &mut frame_input.events);

        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            None,
            &physics_hooks,
            &event_handler,
          );
 
          ball_cue.update(&mut rigid_body_set);
          for i in 0..LAST_BALL {
            balls[i].update(&mut rigid_body_set);
    }

    if shadows_enabled {
        directional0.generate_shadow_map(1024, &balls[0].sphere);
        directional1.generate_shadow_map(1024, &balls[0].sphere);
        spot0.generate_shadow_map(1024, &balls[0].sphere);
    }

    let lights = [
        &ambient as &dyn Light,
        &spot0,
        &directional0,
        &directional1,
        &point0,
        &point1,
    ];

    ball_cue.sphere.material.lighting_model = lighting_model;
    for i in 0..LAST_BALL {
        balls[i].sphere.material.lighting_model = lighting_model;
    }

    let screen = frame_input.screen();
    screen.clear(ClearState::default());
    screen.render(
            &camera,
            pool_hall.into_iter()
            .chain(&table)
            .chain(&ball_cue.sphere)
            .chain(&balls[0].sphere)
            .chain(&balls[1].sphere)
            .chain(&balls[2].sphere)
            .chain(&balls[3].sphere)
            .chain(&balls[4].sphere)
            .chain(&balls[5].sphere)
            .chain(&balls[6].sphere)
            .chain(&balls[7].sphere)
            .chain(&balls[8].sphere)
            .chain(&balls[9].sphere)
            .chain(&balls[10].sphere)
            .chain(&balls[11].sphere)
            .chain(&balls[12].sphere)
            .chain(&balls[13].sphere)
            .chain(&balls[14].sphere)
            .chain(&queue0)
            .chain(&queue1)
            .chain(&queue2),
            &lights);

        if change {
            screen.render(
                &camera,
        ball_cue.sphere.into_iter()
                .chain(&queue0)
                .chain(&queue1)
                .chain(&queue2)
                .chain(&pick_mesh),
        &[&ambient, &directional],
            );
        }

        screen.write(|| gui.render());

        FrameOutput::default()
    });
}