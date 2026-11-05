# Shared argument handling for navigation and relocalization launch wrappers.
prepare_prior_map_args() {
    PRIOR_MAP_PCD="$SLAM_STUDY_ROOT/maps/map.pcd"
    PRIOR_MAP_YAML="$SLAM_STUDY_ROOT/maps/map.yaml"
    PRIOR_MAP_ROS_ARGS=()
    PRIOR_MAP_INSPECT=false
    local pcd_explicit=false yaml_explicit=false arg map_dir map_name

    for arg in "$@"; do
        case "$arg" in
            map_pcd:=*) PRIOR_MAP_PCD="${arg#map_pcd:=}"; pcd_explicit=true ;;
            map_yaml:=*) PRIOR_MAP_YAML="${arg#map_yaml:=}"; yaml_explicit=true ;;
            -s|--show-args|-h|--help)
                PRIOR_MAP_INSPECT=true
                PRIOR_MAP_ROS_ARGS+=("$arg")
                ;;
            *) PRIOR_MAP_ROS_ARGS+=("$arg") ;;
        esac
    done

    # Launch introspection does not need an existing map or a run directory.
    if "$PRIOR_MAP_INSPECT"; then return 0; fi

    if [ -z "$PRIOR_MAP_PCD" ] || [ -z "$PRIOR_MAP_YAML" ]; then
        echo "[错误] map_pcd 和 map_yaml 不能是空路径。" >&2
        return 1
    fi

    # A custom map must not silently pick up the repository's default partner.
    if "$pcd_explicit" && ! "$yaml_explicit"; then
        map_dir="$(dirname -- "$PRIOR_MAP_PCD")"
        map_name="$(basename -- "$PRIOR_MAP_PCD")"
        PRIOR_MAP_YAML="$map_dir/${map_name%.*}.yaml"
    elif "$yaml_explicit" && ! "$pcd_explicit"; then
        map_dir="$(dirname -- "$PRIOR_MAP_YAML")"
        map_name="$(basename -- "$PRIOR_MAP_YAML")"
        PRIOR_MAP_PCD="$map_dir/${map_name%.*}.pcd"
    fi

    if [ ! -f "$PRIOR_MAP_PCD" ] || [ ! -r "$PRIOR_MAP_PCD" ]; then
        echo "[错误] 先验点云不存在或不可读: $PRIOR_MAP_PCD" >&2
        echo "请先运行 ./scripts/mapping.sh，或用 map_pcd:=路径 指定已有地图。" >&2
        return 1
    fi
    if "$yaml_explicit" && { [ ! -f "$PRIOR_MAP_YAML" ] || [ ! -r "$PRIOR_MAP_YAML" ]; }; then
        echo "[错误] 指定的栅格地图配置不存在或不可读: $PRIOR_MAP_YAML" >&2
        return 1
    fi
    # If the inferred YAML is absent, the localizer generates it from the PCD.
}
