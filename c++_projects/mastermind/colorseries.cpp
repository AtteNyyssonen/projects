#include "colorseries.hh"
#include <vector>
#include <iostream>
#include <algorithm>

/* Constructs a ColorSeries object that has an empty vector
 * for the colors saved in the color_str. Also sets the score
 * keeping attributes to zero so that they can be incremented
 * when comparing.
 *
 * @param color_str string that has the color data in form
 *        e.g. BGYV or BBBB.
 * */
ColorSeries::ColorSeries(std::string color_str):
    color_str_(color_str), colors_({}), correct_pos_and_col_(0),
    correct_col_incorrect_pos_(0)
{
    // Saves the chars as elements of a vector colors_.
    // E.g. {B, B, B, B}
    for(auto chars : color_str_) {
        colors_.push_back(chars);
    }
}

ColorSeries::~ColorSeries() {}

std::vector<char> ColorSeries::get_color_series()
{
    return colors_;
}

/* Prints all the info associated with the object.
 * */
void ColorSeries::print_info()
{
    std::cout << "|";
    for(char chars : colors_)
    {
        std::cout << " " << chars;
    }
    std::cout << " " << "|" << " " << correct_pos_and_col_ << " " << "|";
    std::cout << " " << correct_col_incorrect_pos_ << " " << "|" << std::endl;
}

/* Checks if all the colors are in right positions.
 * @return true if everything is correct, else false.
 * */
bool ColorSeries::check_score()
{
    if(correct_pos_and_col_ == 4)
    {
        return true;
    }
    return false;
}

/* Compares the created ColorSeries to the secret and adds points to the
 * initalized scores attributes in the object based on the rules stated
 * in the docstring at the beginning of main.cpp.
 *
 * @param secret the ColorSeries object that has the series that the
 *        user is trying to guess.
 * */
void ColorSeries::compare_to_secret(ColorSeries secret)
{
    // Initialize copy vectors to mutate and iterate;
    std::vector<char> secret_serie = secret.get_color_series();
    std::vector<char> guess_serie = colors_;
    // Iterates through every char of the guessed colors.
    // If the indexes are same add score to totally correct.
    for(std::vector<int>::size_type i=0; i < guess_serie.size(); ++i)
    {
        if(guess_serie.at(i) == secret_serie.at(i)) {
            correct_pos_and_col_ += 1;
        }
    }
    // Iterate through the guess and the secret
    for(std::vector<int>::size_type i=0; i < guess_serie.size(); ++i)
    {
        for(std::vector<int>::size_type j=0; j < secret_serie.size(); ++j)
        {
            // If there is a char in a different place add score
            // and mutate it so it doesn't get counted twice
            if(secret_serie.at(i) == guess_serie.at(j))
            {
                correct_col_incorrect_pos_ += 1;
                guess_serie.at(j) = 'F';
                break;
            }
        }
    }
    // The above iteration also adds the totally correct to the
    // partially correct so to get the actual value of the
    // partially we remove the totally corrects
    correct_col_incorrect_pos_ -= correct_pos_and_col_;
}
