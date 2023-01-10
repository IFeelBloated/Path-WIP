#pragma once
#include "scene/scene.h"

namespace Samplers {
	using Signature = auto(const glm::vec3&, const glm::vec3&)->std::tuple<glm::vec3, double>;
	using Ǝ = std::function<Signature>;

	constexpr auto InvalidDensity = -std::numeric_limits<double>::infinity();
}

namespace Samplers::InternalState {
	inline auto Seeder = std::random_device{};
	inline auto SequenceGenerator = std::mt19937{ Seeder() };
}

namespace Samplers::Standard {
	auto Uniform(auto LowerBound, auto UpperBound) {
		return [=, Sampler = std::uniform_real_distribution{ LowerBound, UpperBound }] mutable {
			return Sampler(InternalState::SequenceGenerator);
		};
	}
}

namespace Samplers::Hemispherical::PDFs {
	auto CosineWeighted(auto cosθ, auto Concentration) {
		return (Concentration + 1) / (2 * std::numbers::pi) * std::pow(cosθ, Concentration);
	}
}

namespace Samplers::Hemispherical {
	auto CosineWeighted(auto&& AxisGenerator, auto Concentration) { // might produce invalid ωi samples if the axis is not the surface normal
		return [=, φSampler = Standard::Uniform(0., 1.), θSampler = Standard::Uniform(0., 1.)](auto&& ...Arguments) mutable {
			auto PolarToCartesian = [](auto Radius, auto θ) { return std::array{ Radius * std::cos(θ), Radius * std::sin(θ) }; };
			auto [φ, cosθ] = std::array{ 2 * std::numbers::pi * φSampler(), std::pow(θSampler(), 1. / (Concentration + 1)) };
			auto [y, LongitudeRadius] = PolarToCartesian(1., std::acos(cosθ));
			auto [x, z] = PolarToCartesian(LongitudeRadius, φ);
			auto Axis = AxisGenerator(Arguments...);
			auto SupportAxis = glm::normalize(std::abs(Axis.x) > std::abs(Axis.y) ? glm::vec3{ Axis.z, 0, -Axis.x } : glm::vec3{ 0, -Axis.z, Axis.y });
			return std::tuple{ glm::mat3{ glm::cross(Axis, SupportAxis), Axis, SupportAxis } * glm::vec3{ x, y, z }, PDFs::CosineWeighted(cosθ, Concentration) };
		};
	}
	auto CosineWeighted(auto Concentration) { return CosineWeighted([](auto&& SurfaceNormal, auto&&...) { return SurfaceNormal; }, Concentration); }
	inline auto Uniform = CosineWeighted(0.);
	inline auto DiffuseImportance = CosineWeighted(1.);
	auto SpecularImportance(auto SpecularExponent) {
		return [=, Sampler = CosineWeighted([](auto&& SurfaceNormal, auto&& ωo) { return Ray::Reflect(-ωo, SurfaceNormal); }, SpecularExponent)](auto&& SurfaceNormal, auto&& ωo) mutable {
			if (auto [ωi, ProbabilityDensity] = Sampler(SurfaceNormal, ωo); glm::dot(ωi, SurfaceNormal) >= 0)
				return std::tuple{ ωi, ProbabilityDensity };
			return std::tuple{ glm::vec3{}, InvalidDensity };
		};
	}
	auto PhongMultipleImportance(auto SpecularExponent) {
		return [=, RussianRoulette = Standard::Uniform(0., 1.), DiffuseSampler = DiffuseImportance, SpecularSampler = SpecularImportance(SpecularExponent)](auto&& SurfaceNormal, auto&& ωo) mutable {
			auto PowerHeuristics = [&, β = 2](auto&& Sampler, auto&& AuxiliaryPDF) {
				if (auto [ωi, ProbabilityDensity] = Sampler(SurfaceNormal, ωo); ProbabilityDensity == InvalidDensity)
					return std::tuple{ glm::vec3{}, InvalidDensity };
				else
					return std::tuple{ ωi, (std::pow(ProbabilityDensity, β) + std::pow(AuxiliaryPDF(ωi), β)) * ProbabilityDensity / (2 * std::pow(ProbabilityDensity, β)) };
			};
			if (RussianRoulette() < 0.5)
				return PowerHeuristics(SpecularSampler, [&](auto&& ωi) { return PDFs::CosineWeighted(glm::dot(ωi, SurfaceNormal), 1.); });
			return PowerHeuristics(DiffuseSampler, [&](auto&& ωi) { return PDFs::CosineWeighted(std::max(glm::dot(Ray::Reflect(-ωo, SurfaceNormal), ωi), 0.f), SpecularExponent); });
		};
	}
}

namespace Samplers::δ {
	inline auto Reflective = [](auto&& SurfaceNormal, auto&& ωo) { return std::tuple{ Ray::Reflect(-ωo, SurfaceNormal), 1. }; };
	auto Dielectric(auto η, auto&& NormalPerturbator) requires requires { { NormalPerturbator(glm::vec3{}) }->SubtypeOf<glm::vec3>; } {
		return [=, RussianRoulette = Standard::Uniform(0., 1.)](auto&& SurfaceNormal, auto&& ωo) mutable {
			auto [Reflectance, IncidentNormal, Incidentη, IsExteriorSurface] = [&] {
				auto cosθi = glm::dot(-ωo, SurfaceNormal);
				auto IncidenceInfo = cosθi > 0 ? std::tuple{ -NormalPerturbator(SurfaceNormal), η, false } : std::tuple{ NormalPerturbator(SurfaceNormal), 1 / η, true };
				auto [η1, η2] = cosθi > 0 ? std::tuple{ 1., η } : std::tuple{ η, 1. };
				if (auto sinθt = η2 / η1 * std::sqrt(std::max(0., 1. - cosθi * cosθi)); sinθt >= 1)
					return std::tuple{ 1. } + IncidenceInfo;
				else {
					auto cosθt = std::sqrt(std::max(0., 1. - sinθt * sinθt));
					auto RootOfRs = (η1 * std::abs(cosθi) - η2 * cosθt) / (η1 * std::abs(cosθi) + η2 * cosθt);
					auto RootOfRp = (η2 * std::abs(cosθi) - η1 * cosθt) / (η2 * std::abs(cosθi) + η1 * cosθt);
					return std::tuple{ (RootOfRs * RootOfRs + RootOfRp * RootOfRp) / 2 } + IncidenceInfo;
				}
			}();
			if (auto ReferenceNormal = IsExteriorSurface ? SurfaceNormal : -SurfaceNormal; RussianRoulette() < Reflectance) {
				if (auto ωi = Ray::Reflect(-ωo, IncidentNormal); IsExteriorSurface && glm::dot(ωi, ReferenceNormal) >= 0)
					return std::tuple{ ωi, 1. };
			}
			else if (auto ωi = Ray::Refract(-ωo, IncidentNormal, Incidentη); glm::dot(ωi, ReferenceNormal) < 0)
				return std::tuple{ ωi, 1. };
			return std::tuple{ glm::vec3{}, InvalidDensity };
		};
	}
	auto Dielectric(auto η) { return Dielectric(η, [](auto&& x) { return x; }); }
	auto Dielectric(auto η, Real auto SpecularExponent) {
		return Dielectric(η, [=, Sampler = Hemispherical::CosineWeighted(SpecularExponent)](auto&& SurfaceNormal) mutable {
			return std::get<0>(Sampler(SurfaceNormal));
		});
	}
}

namespace BSDFs {
	using Signature = auto(const glm::vec3&, const glm::vec3&, const glm::vec3&)->glm::vec3;
	using Ǝ = std::function<Signature>;

	auto Diffuse(auto&& DiffuseCoefficients) {
		return [=](auto&&...) { return DiffuseCoefficients / std::numbers::pi; };
	}
	auto Specular(auto&& SpecularCoefficients, auto SpecularExponent) {
		return [=](auto&& SurfaceNormal, auto&& ωi, auto&& ωo) {
			return std::pow(std::max(glm::dot(Ray::Reflect(-ωi, SurfaceNormal), ωo), 0.f), SpecularExponent) * (SpecularExponent + 2) / (2 * std::numbers::pi * glm::dot(ωi, SurfaceNormal)) * SpecularCoefficients;
		};
	}
	auto Phong(auto&& DiffuseCoefficients, auto&& SpecularCoefficients, auto SpecularExponent) {
		auto TotalReflectance = DiffuseCoefficients + SpecularCoefficients;
		auto [DiffuseReflectance, SpecularReflectance] = std::tuple{ DiffuseCoefficients * DiffuseCoefficients / TotalReflectance, SpecularCoefficients * SpecularCoefficients / TotalReflectance };
		return [=, DiffuseBRDF = Diffuse(DiffuseReflectance), SpecularBRDF = Specular(SpecularReflectance, SpecularExponent)](auto&& SurfaceNormal, auto&& ωi, auto&& ωo) {
			return DiffuseBRDF() + SpecularBRDF(SurfaceNormal, ωi, ωo);
		};
	}
	auto δ(auto&& SpecularCoefficients) { // only works with δ samplers
		return [=](auto&& SurfaceNormal, auto&& ωi, auto&&...) { return SpecularCoefficients / glm::dot(ωi, SurfaceNormal); };
	}
	auto δ(auto&& SpecularCoefficients, auto&& TransmittanceCoefficients) { // only works with δ samplers
		return [=](auto&& SurfaceNormal, auto&& ωi, auto&& ωo) {
			if (auto ReferenceNormal = glm::dot(-ωo, SurfaceNormal) <= 0 ? SurfaceNormal : -SurfaceNormal; glm::dot(ωi, ReferenceNormal) >= 0)
				return SpecularCoefficients / glm::dot(ωi, SurfaceNormal);
			return TransmittanceCoefficients / glm::dot(ωi, SurfaceNormal);
		};
	}
}












namespace Ray {
	inline auto RecursiveTracingProbability = 0.9;

	auto Trace(auto&& EyePoint, auto&& RayDirection, auto&& ObjectRecords)->glm::vec3 {

		static auto RussianRoulette = Samplers::Standard::Uniform(0., 1.);


		if (auto&& [t, SurfaceNormal, SurfaceMaterial] = Intersect(EyePoint, RayDirection, ObjectRecords); t != NoIntersection) {
			
			if (glm::dot(RayDirection, SurfaceNormal) > 0 && !SurfaceMaterial.IsDielectric)
				return { 0, 0, 0 };
			
			auto IntersectionPosition = EyePoint + t * RayDirection;
			auto AccumulatedIntensity = SurfaceMaterial.EmissiveIntensity;

			if (RussianRoulette() < RecursiveTracingProbability)
				if (auto [ωi, ProbabilityDensity] = SurfaceMaterial.Sampler(SurfaceNormal, -RayDirection); ProbabilityDensity != Samplers::InvalidDensity) {

					auto attenuate = SurfaceMaterial.BSDF(SurfaceNormal, ωi, -RayDirection);

					auto v = attenuate * Trace(IntersectionPosition + SelfIntersectionDisplacement * ωi, ωi, ObjectRecords) * glm::dot(ωi, SurfaceNormal) / (ProbabilityDensity * RecursiveTracingProbability);

					if (!std::isnan(v[0] + v[1] + v[2]))
						AccumulatedIntensity += v;
				}


			// if (AccumulatedIntensity.x < 0 || AccumulatedIntensity.y < 0 || AccumulatedIntensity.z < 0)
				// std::cout << "?????????????????????????????????????????" << std::endl;

			return AccumulatedIntensity;
			
		}


		//auto t = 0.5 * (RayDirection.y + 1.0);
		//auto bg = (1 - t) * glm::vec3{ 1, 1, 1 } + t * glm::vec3{ .5, .7, 1. };

		auto bg = glm::vec3{ 0, 0, 0 };

		return bg;
	}
}


























// MIS PDF: (DiffuseMIS + SpecMIS) / 2