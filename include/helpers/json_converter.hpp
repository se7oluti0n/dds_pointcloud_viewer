#ifndef JSON_CONVERTER_HPP
#define JSON_CONVERTER_HPP

#include <nlohmann/json.hpp>
#include "Slam3DIDL.hpp"


using json = nlohmann::json;


// Convert PointField to JSON
void to_json(json& j, const PointCloudData::PointField& p) {
    j = json{
        {"name", p.name()},
        {"offset", p.offset()},
        {"datatype", p.datatype()},
        {"count", p.count()}
    };
}

// Convert JSON to PointField
void from_json(const json& j, PointCloudData::PointField& p) {
    j.at("name").get_to(p.name());
    j.at("offset").get_to(p.offset());
    j.at("datatype").get_to(p.datatype());
    j.at("count").get_to(p.count());
}

// Convert PointCloud2 to JSON
void to_json(json& j, const PointCloudData::PointCloud2& p) {
    json fields = json::array();
    for (const auto& field : p.fields()) {
        json field_json;
        to_json(field_json, field);
        fields.push_back(field_json);
    }
    j = json{
        {"timestamp", p.timestamp()},
        {"frame_id", p.frame_id()},
        {"height", p.height()},
        {"width", p.width()},
        {"fields", fields},
        {"is_bigendian", p.is_bigendian()},
        {"point_step", p.point_step()},
        {"row_step", p.row_step()},
        {"data", p.data()},
        {"is_dense", p.is_dense()}
    };
}

// Convert JSON to PointCloud2
void from_json(const json& j, PointCloudData::PointCloud2& p) {
    j.at("timestamp").get_to(p.timestamp());
    j.at("frame_id").get_to(p.frame_id());
    j.at("height").get_to(p.height());
    j.at("width").get_to(p.width());
    const auto& fields = j.at("fields");
    p.fields().resize(fields.size());
    for (int i = 0; i < fields.size(); ++i) {
        fields[i]["name"].get_to(p.fields()[i].name());
        fields[i]["offset"].get_to(p.fields()[i].offset());
        fields[i]["datatype"].get_to(p.fields()[i].datatype());
        fields[i]["count"].get_to(p.fields()[i].count());
    }
    j.at("is_bigendian").get_to(p.is_bigendian());
    j.at("point_step").get_to(p.point_step());
    j.at("row_step").get_to(p.row_step());
    j.at("data").get_to(p.data());
    j.at("is_dense").get_to(p.is_dense());
}

// Convert Vector3D to JSON
void to_json(json& j, const Common::Vector3D& v) {
    j = json{
        {"x", v.x()},
        {"y", v.y()},
        {"z", v.z()}
    };
}

// Convert JSON to Vector3D
void from_json(const json& j, Common::Vector3D& v) {
    j.at("x").get_to(v.x());
    j.at("y").get_to(v.y());
    j.at("z").get_to(v.z());
}

// Convert Quaternion to JSON
void to_json(json& j, const Common::Quaternion& q) {
    j = json{
        {"x", q.x()},
        {"y", q.y()},
        {"z", q.z()},
        {"w", q.w()}
    };
}

// Convert JSON to Quaternion
void from_json(const json& j, Common::Quaternion& q) {
    j.at("x").get_to(q.x());
    j.at("y").get_to(q.y());
    j.at("z").get_to(q.z());
    j.at("w").get_to(q.w());
}

// Convert Pose3D to JSON
void to_json(json& j, const Common::Pose3D& p) {
    json translation;
    json rotation;
    to_json(translation, p.translation());
    to_json(rotation, p.rotation());
    j = json{
        {"translation", translation},
        {"rotation", rotation}
    };
}

// Convert JSON to Pose3D
void from_json(const json& j, Common::Pose3D& p) {
    from_json(j.at("translation"), p.translation());
    from_json(j.at("rotation"), p.rotation());
}

// Convert Pose3DTimestamped to JSON
void to_json(json& j, const Common::Pose3DTimestamped& p) {
    json pose;
    to_json(pose, p.pose());
    j = json{
        {"timestamp", p.timestamp()},
        {"pose", pose}
    };
}

// Convert JSON to Pose3DTimestamped
void from_json(const json& j, Common::Pose3DTimestamped& p) {
    j.at("timestamp").get_to(p.timestamp());
    from_json(j.at("pose"), p.pose());
}

// Convert Keyframe to JSON
void to_json(json& j, const Slam3D::Keyframe& k) {
    json pose;
    json pointcloud;
    to_json(pose, k.pose());
    to_json(pointcloud, k.pointcloud());
    j = json{
        {"keyframe_id", k.keyframe_id()},
        {"pose", pose},
        {"pointcloud", pointcloud}
    };
}

// Convert JSON to Keyframe
void from_json(const json& j, Slam3D::Keyframe& k) {
    j.at("keyframe_id").get_to(k.keyframe_id());
    from_json(j.at("pose"), k.pose());
    from_json(j.at("pointcloud"), k.pointcloud());
}




#endif // JSON_CONVERTER_HPP