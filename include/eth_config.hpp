//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     EthConfig data = nlohmann::json::parse(jsonString);

#pragma once

#include "json.hpp"

#include <optional>
#include <stdexcept>
#include <regex>

#ifndef NLOHMANN_OPT_HELPER
#define NLOHMANN_OPT_HELPER
namespace nlohmann {
    template <typename T>
    struct adl_serializer<std::shared_ptr<T>> {
        static void to_json(json & j, const std::shared_ptr<T> & opt) {
            if (!opt) j = nullptr; else j = *opt;
        }

        static std::shared_ptr<T> from_json(const json & j) {
            if (j.is_null()) return std::unique_ptr<T>(); else return std::unique_ptr<T>(new T(j.get<T>()));
        }
    };
}
#endif

namespace quicktype {
    using nlohmann::json;

    #ifndef NLOHMANN_UNTYPED_quicktype_HELPERHELPER
    #define NLOHMANN_UNTYPED_quicktype_HELPERHELPER
    inline json get_untyped(const json & j, const char * property) {
        if (j.find(property) != j.end()) {
            return j.at(property).get<json>();
        }
        return json();
    }

    inline json get_untyped(const json & j, std::string property) {
        return get_untyped(j, property.data());
    }
    #endif

    #ifndef NLOHMANN_OPTIONAL_quicktype_
    #define NLOHMANN_OPTIONAL_quicktype_
    template <typename T>
    inline std::shared_ptr<T> get_optional(const json & j, const char * property) {
        if (j.find(property) != j.end()) {
            return j.at(property).get<std::shared_ptr<T>>();
        }
        return std::shared_ptr<T>();
    }

    template <typename T>
    inline std::shared_ptr<T> get_optional(const json & j, std::string property) {
        return get_optional<T>(j, property.data());
    }
    #endif

    class EthConfig {
        public:
        EthConfig() = default;
        virtual ~EthConfig() = default;

        private:
        std::shared_ptr<std::vector<int64_t>> ip;

        public:
        std::shared_ptr<std::vector<int64_t>> get_ip() const { return ip; }
        void set_ip(std::shared_ptr<std::vector<int64_t>> value) { this->ip = value; }
    };
}

namespace quicktype {
    void from_json(const json & j, EthConfig & x);
    void to_json(json & j, const EthConfig & x);

    inline void from_json(const json & j, EthConfig& x) {
        x.set_ip(get_optional<std::vector<int64_t>>(j, "ip"));
    }

    inline void to_json(json & j, const EthConfig & x) {
        j = json::object();
        j["ip"] = x.get_ip();
    }
}
