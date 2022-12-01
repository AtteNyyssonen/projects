#ifndef COLORSERIES_HH
#define COLORSERIES_HH

#include <string>
#include <vector>

class ColorSeries;
using ColorSeriesContainer = std::vector< ColorSeries >;

class ColorSeries
{
public:

    ColorSeries(std::string color_str);
    ~ColorSeries();

    // Compares a color serie to the secret serie and
    // adds to the scores based on the comparison
    void compare_to_secret(ColorSeries secret);

    // Gets the colors saved into a serie object
    std::vector<char> get_color_series();

    // Prints all the data from a serie object in the wanted format
    // Iterated for every serie object
    void print_info();

    // Checks if all the colors from the secret
    // have been guessed right
    bool check_score();

private:

    // Inputted colors as a string and
    // Vector initalization for chars of the color string
    std::string color_str_;
    std::vector<char> colors_;

    // Variables for keeping score
    int correct_pos_and_col_;
    int correct_col_incorrect_pos_;
};

#endif // COLORSERIES_HH
