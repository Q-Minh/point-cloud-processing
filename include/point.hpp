#pragma once

struct point_t
{
	float x = 0.f, y = 0.f, z = 0.f;

	friend point_t operator*(float k, point_t p)
	{
		return point_t
		{
			k * p.x,
			k * p.y,
			k * p.z,
		};
	}

	friend point_t operator/(point_t p, float k)
	{
		return point_t
		{
			p.x / k,
			p.y / k,
			p.z / k
		};
	}

	bool operator==(point_t const& p) const
	{
		float constexpr e = std::numeric_limits<float>::epsilon();
		auto const dx = std::abs(x - p.x);
		auto const dy = std::abs(y - p.y);
		auto const dz = std::abs(z - p.z);
		auto const equals = dx < e&& dy < e&& dz < e;
		return equals;
	};

	bool operator!=(point_t const& p) const
	{
		return !(*this == p);
	}

	point_t operator+(point_t const& other) const
	{
		return point_t
		{
			x + other.x,
			y + other.y,
			z + other.z
		};
	}

	point_t operator-(point_t const& other) const
	{
		return point_t
		{
			x - other.x,
			y - other.y,
			z - other.z
		};
	}
};
