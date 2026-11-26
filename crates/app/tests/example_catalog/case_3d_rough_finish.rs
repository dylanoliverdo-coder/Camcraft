// Provenance: original CamCraft 3D rough-and-finish catalog contract (#2866).
mod catalog_3d_rough_finish {
    use super::*;

    #[test]
    fn a_saved_finish_cache_requires_regeneration_before_posting() {
        use camcraft_app::{model, operation, posting, project, world::new_world};

        let path = repo("examples/catalog/3d/rough-finish/rough-finish.camproj");
        let world = new_world();
        project::load(&world, &path).expect("the saved 3D example opens");
        for name in ["3D Finish Parallel", "3D Finish Waterline"] {
            let op = model::find_by_name(&world, name).expect("finish operation");
            assert!(model::toolpath_of(&world, &op).is_some(), "{name}: saved cache");
            assert_eq!(operation::status_of(&world, &op), operation::OpStatus::Stale, "{name}");
        }
        let pocket = model::find_by_name(&world, "3D Pocket1").expect("pocket operation");
        assert_eq!(operation::status_of(&world, &pocket), operation::OpStatus::Ok);
        let err = posting::post_setup_unwritten(&world).expect_err("pre-#4062 finish caches must not post");
        assert!(err.contains("3D Finish") && err.contains("regenerate"), "{err}");
    }
    #[test]
    fn regenerates() {
        run_case("3d", "rough-finish", "regenerates", regenerate);
    }
    #[test]
    fn rebuilds() {
        run_case("3d", "rough-finish", "rebuilds", rebuild);
    }
}
