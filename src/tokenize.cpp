#include <catch.hpp>

#include <tokenize.hpp>

SCENARIO("string tokenization", "[tokenization]") {
	GIVEN("identical sentences with different spacing and a newline") {
		std::string const s1 = "octrees are very fast\n";
		std::string const s2 = "octrees are very   fast \n";
		std::string const s3 = "  octrees are  very   fast \n";

		WHEN("tokenizing sentences") {
			auto const tokens1 = tokenize(s1);
			auto const tokens2 = tokenize(s2);
			auto const tokens3 = tokenize(s3);
			THEN("tokenization yields the same tokens") {
				std::vector<std::string> truth = { "octrees", "are", "very", "fast" };

				REQUIRE(tokens1.size() == truth.size());
				REQUIRE(tokens2.size() == truth.size());
				REQUIRE(tokens3.size() == truth.size());
				REQUIRE(tokens1 == truth);
				REQUIRE(tokens2 == truth);
				REQUIRE(tokens3 == truth);
			}
		}
	}
}