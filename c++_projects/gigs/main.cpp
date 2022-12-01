/* COMP.CS.110 Project 2: GIGS
 *
 * This is a program that works as a simple gig calendar where you can store
 * info on upcoming gigs by following the strictly given commands listed below.
 *
 * Necessary params are shown enclosed in < .... >
 * If any of the parameters have multiple words e.g. band names they need to be
 * surrounded by quotation marks   "... ..."
 *
 * ARTISTS - Prints all known artist names in alphabetical order.
 * ARTIST <artist name> - Prints single artist's gigs in order by date.
 * STAGES - Prints all known stage names and their locations in alphabetical
 * order.
 * STAGE <stage name> - Prints all artists having a gig at the given stage in
 * alphabetical order if the stage exists.
 * ADD_ARTIST <artist name> - Adds a new artist with emtpy gig data.
 * ADD_GIG <artist name> <date> <town name> <stage name> - Adds a new gig for
 * an artist with the given date, town, and stage. There can't already
 * be a gig for the artist on the same date. There also can't be other gigs at
 * the same stage at the same time.
 * CANCEL <artist name> <date> - Cancels artist's gigs after the given date if
 * the date is a valid one and there are gigs to cancel.
 * QUIT exits the program.
 *
 * A data file can be used to insert info into the calendar but there are some
 * strict restrictions that need to be followed.
 *
 * The data file's lines are expected to follow this format:
 *
 * artist_name;date;town_name;stage_name
 *
 * The program expects there to be no mistakes in the data file e.g. multiple
 * shows on the same date. It will only check the validity of the format listed
 * above. The program will terminate if there aren't lines with 4 parts divided
 * by ';'.
 *
 *
 * */

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <map>
#include <fstream>

// Farthest year for which gigs can be allocated
const std::string FARTHEST_POSSIBLE_YEAR = "2030";

// Map of all the accepted commands and the amount of necessary
// parameters needed + the command itself.
const std::map<std::string, int> ACCEPTED_COMMANDS = {
    {"ARTISTS", 1}, {"ARTIST", 2}, {"STAGES", 1}, {"STAGE", 2},
    {"ADD_ARTIST", 2}, {"ADD_GIG", 5}, {"CANCEL", 3}, {"QUIT", 1}
};

// Casual split func, if delim char is between "'s, ignores it.
std::vector<std::string> split(const std::string& str, char delim = ';')
{
    std::vector<std::string> result = {""};
    bool inside_quotation = false;
    for ( char current_char : str )
    {
        if ( current_char == '"' )
        {
            inside_quotation = not inside_quotation;
        }
        else if ( current_char == delim and not inside_quotation )
        {
            result.push_back("");
        }
        else
        {
            result.back().push_back(current_char);
        }
    }
    if ( result.back() == "" )
    {
        result.pop_back();
    }
    return result;
}

// Checks if the given date_str is a valid date, i.e. if it has the format
// dd-mm-yyyy and if it is also otherwise possible date.
bool is_valid_date(const std::string& date_str)
{
    std::vector<std::string> date_vec = split(date_str, '-');
    if ( date_vec.size() != 3 )
    {
        return false;
    }

    std::string year = date_vec.at(0);
    std::string month = date_vec.at(1);
    std::string day = date_vec.at(2);
    std::vector<std::string> month_sizes
            = { "31", "28", "31", "30", "31", "30",
                "31", "31", "30", "31", "30", "31" };

    if ( month < "01" or month > "12" )
    {
        return false;
    }
    if ( year < "0001" or year > FARTHEST_POSSIBLE_YEAR )
    {
        return false;
    }
    unsigned int year_int = stoi(year);
    bool is_leap_year = (year_int % 400 == 0) or
                        (year_int % 100 != 0 and year_int % 4 == 0);
    if ( day >= "01" and
        (day <= month_sizes.at(stoi(month) - 1) or
        (month == "02" and is_leap_year and day <= "29")) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Adds a gig into the container. Checks that there isnt a gig at the same date
// Or the same venue. Returns false if there is an issue with the booking
// so that the program terminates while reading a file.
bool add_gig(std::string name, std::string date, std::string city,
             std::string stage,
             std::map<std::string,
             std::vector<std::vector<std::string>>>& gig_data)
{
    auto itr = gig_data.find(name);
    if (itr != gig_data.end())
    {
        std::vector<std::vector<std::string>>& gig_vec = itr -> second;
        for(auto& gigs : gig_vec)
        {
            if(gigs.at(0) == date)
            {
                std::cout << "Error: Already exists." << std::endl;
                return false;

            }
            for (auto others = gig_data.begin(); others !=
                 gig_data.end(); ++others)
            {
                std::vector<std::vector<std::string>> gig_vecs =
                        others -> second;
                for (auto& gig : gig_vecs)
                {
                    if(gig.at(0) == date && gig.at(1) == city
                            && gig.at(2) == stage)
                    {
                        std::cout << "Error: Already exists." << std::endl;
                        return false;

                    }
                }
            }
        }
        std::vector<std::string> gig = {date, city, stage};
        gig_vec.push_back(gig);
        std::sort(gig_vec.begin(), gig_vec.end());
    }
    else
    {
        for (auto others = gig_data.begin(); others !=
             gig_data.end(); ++others)
        {
            std::vector<std::vector<std::string>> gig_vecs =
                    others -> second;
            for (auto& gig : gig_vecs)
            {
                if(gig.at(0) == date && gig.at(1) == city
                        && gig.at(2) == stage)
                {
                    std::cout << "Error: Already exists." << std::endl;
                    return false;
                }
            }
        }
        std::vector<std::string> data_vec = {date, city, stage};
        gig_data.insert({name, {data_vec}});
    }
    return true;
}

// Reads data from file and saves it into the container.
// returns false if there was a error with reading the file
// else returns true.
bool save_data(std::string filename,
               std::map<std::string,
               std::vector<std::vector<std::string>>>& gig_data)
{
    std::ifstream file_obj(filename);
    std::string line;
    if (not file_obj)
    {
        std::cout << "Error: File could not be read." << std::endl;
        return false;
    }
    while (getline(file_obj, line))
    {
        std::vector<std::string> row_data = split(line);
        if (row_data.size() != 4)
        {
            std::cout << "Error: Invalid format in file." << std::endl;
            return false;
        }
        for (auto& part : row_data)
        {
            if (part.size() == 0)
            {
                std::cout << "Error: Invalid format in file." << std::endl;
                return false;
            }

        }
        if(!is_valid_date(row_data.at(1)))
        {
            std::cout << "Error: Invalid date." << std::endl;
            return false;
        }
        std::string name = row_data.at(0);
        std::string date = row_data.at(1);
        std::string city = row_data.at(2);
        std::string stage = row_data.at(3);
        bool check = add_gig(name, date, city, stage, gig_data);
        if(!check)
        {
            return false;
        }
    }
    file_obj.close();
    return true;
}

// Checks that the user input is a valid command and that
// it has the right amount of params.
// returns true if the criteria are met, else returns false.
bool check_user_input(std::vector<std::string>& input_elements)
{
    if(input_elements.size() == 0)
    {
        return false;
    }
    std::transform(input_elements.at(0).begin(), input_elements.at(0).end(),
                   input_elements.at(0).begin(), ::toupper);
    auto itr = ACCEPTED_COMMANDS.find(input_elements.at(0));
    if(itr != ACCEPTED_COMMANDS.end())
    {
        unsigned int necessary_params = itr -> second;
        if (input_elements.size() >= necessary_params)
        {
            return true;
        }
    }
    return false;
}

// Checks that the user inputted data has a artist that can be found in the
// container, a valid date and that no one else has a gig in that exact location
// on the same day. If these criteria are met the function adds the gig.
void check_user_and_add(std::map<std::string,
                       std::vector<std::vector<std::string>>>& gig_data,
                       std::vector<std::string> input_elements)
{
    std::string artist = input_elements.at(1);
    std::string date = input_elements.at(2);
    std::string location = input_elements.at(3);
    std::string venue = input_elements.at(4);
    if(gig_data.find(artist) == gig_data.end())
    {
        std::cout << "Error: Not found." << std::endl;
        return;
    }
    if(!is_valid_date(date))
    {
        std::cout << "Error: Invalid date." << std::endl;
        return;
    }
    bool check = add_gig(artist, date, location, venue, gig_data);
    if(check)
    {
        std::cout << "Gig added." << std::endl;
    }
}

// Cancels artists gigs after the given date. First makes sure that the
// artist is in the container and the date is valid.
void check_and_cancel(std::map<std::string,
                      std::vector<std::vector<std::string>>>& gig_data,
                      std::vector<std::string> input_elements)
{
    std::string artist = input_elements.at(1);
    std::string date = input_elements.at(2);
    auto itr = gig_data.find(artist);
    if(itr == gig_data.end())
    {
        std::cout << "Error: Not found." << std::endl;
        return;
    }
    if(!is_valid_date(date))
    {
        std::cout << "Error: Invalid date." << std::endl;
        return;
    }
    std::vector<std::vector<std::string>>& gig_vecs = itr -> second;
    for(unsigned long i = 0; i < gig_vecs.size();)
    {
        if(gig_vecs.at(i).at(0) < date)
        {
            ++i;
        }
        else if(gig_vecs.at(i).at(0) > date)
        {
            gig_vecs.erase(gig_vecs.begin()+i, gig_vecs.end());
            std::cout <<"Artist's gigs after the given date cancelled."
                        << std::endl;
            return;
        }
        else
        {
            // If the given date was further ahead than any in the gig vectors
            // we print the error message
            std::cout << "Error: No gigs after the given date." << std::endl;
            return;
        }
    }
}

// Prints all the gigs found from a certain user inputted artist.
void print_artist(std::map<std::string,
                  std::vector<std::vector<std::string>>>& gig_data,
                  std::vector<std::string> input_elements)
{
    auto itr = gig_data.find(input_elements.at(1));
    if(itr == gig_data.end())
    {
        std::cout << "Error: Not found." << std::endl;
        return;
    }
    std::cout << "Artist " << input_elements.at(1) << " has the following ";
    std::cout << "gigs in the order they are listed:" << std::endl;

    std::vector<std::vector<std::string>> gigs = itr -> second;
    // Gig vectors should be already sorted by date and only have 3 element
    // members that follow the hardcoded pattern
    for(auto& gig_vec : gigs)
    {
        std::cout << " - " << gig_vec.at(0) << " : " << gig_vec.at(1);
        std::cout << ", " << gig_vec.at(2) << std::endl;
    }
}

// Prints all the stages found in the gig vectors.
void print_stages(std::map<std::string,
                  std::vector<std::vector<std::string>>>& gig_data)
{
    // collects every city-stage pair found
    std::vector<std::vector<std::string>> city_stage;
    for (auto itr = gig_data.begin(); itr != gig_data.end(); ++itr)
    {
        std::vector<std::vector<std::string>> gig_vecs = itr -> second;
        for (auto& gig : gig_vecs)
        {
            std::vector<std::string> venue_info = {gig.at(1), gig.at(2)};
            city_stage.push_back(venue_info);

        }
    }
    // sorts and deletes every duplicate from the temporary vector
    // before the info is printed out
    std::sort(city_stage.begin(), city_stage.end());
    for (unsigned long i = 0; i < city_stage.size(); ++i)
    {
        std::string city = city_stage.at(i).at(0);
        std::string venue = city_stage.at(i).at(1);
        for (unsigned long j = 1+i; j < city_stage.size(); ++j)
        {
            std::string next_city = city_stage.at(j).at(0);
            std::string next_venue = city_stage.at(j).at(1);
            if (city == next_city && venue == next_venue)
            {
                city_stage.erase(city_stage.begin() + j);
                --i;
                --j;
            }
        }
    }
    std::cout << "All gig places in alphabetical order: " << std::endl;
    for (const auto& vecs : city_stage)
    {
        std::cout << vecs.at(0) << ", " << vecs.at(1) << std::endl;
    }
}

// Prints all the artists that are performing at the user inputted stage.
void print_stage(std::map<std::string,
                 std::vector<std::vector<std::string>>>& gig_data,
                 std::vector<std::string> input_elements)
{
    // collects every artist that has the user inputted stage
    // into the temp vector.
    std::vector<std::string> artists;
    for (auto itr = gig_data.begin(); itr != gig_data.end(); ++itr)
    {
        std::vector<std::vector<std::string>> gig_vecs = itr -> second;
        for(auto& gig_vec : gig_vecs)
        {
            if(input_elements.at(1) == gig_vec.at(2))
            {
                artists.push_back(itr -> first);
            }
        }
    }
    if(artists.size() == 0)
    {
        std::cout << "Error: Not found." << std::endl;
        return;
    }
    std::sort(artists.begin(), artists.end());
    std::cout << "Stage " << input_elements.at(1) << " has gigs of the ";
    std::cout <<"following artists:" << std::endl;
    for(auto& artist : artists)
    {
        std::cout << " - " << artist << std::endl;
    }
}

// Calls print functions based on the users selection.
void print(std::map<std::string,
           std::vector<std::vector<std::string>>>& gig_data,
           std::vector<std::string> input_elements, std::string selection)
{
    if(selection == "ARTISTS")
    {
        std::cout << "All artists in alphabetical order:" << std::endl;
        for (auto& itr : gig_data)
        {
            std::cout << itr.first << std::endl;
        }
    }
    else if (selection == "ARTIST")
    {
        print_artist(gig_data, input_elements);
    }
    else if (selection == "STAGES")
    {
        print_stages(gig_data);
    }
    else if (selection == "STAGE")
    {
        print_stage(gig_data, input_elements);
    }
}

// Adds artist with empty gigs vector if the artist isn't
// already in the container.
void add_artist(std::map<std::string,
                std::vector<std::vector<std::string>>>& gig_data,
                std::string artist)
{
    if(gig_data.find(artist) == gig_data.end())
    {
        gig_data.insert({artist, {}});
        std::cout << "Artist added." << std::endl;
        return;
    }
    std::cout << "Error: Already exists." << std::endl;
}

// Interface for all the possible commands.
// Calls functions based on user inputs.
// only returns false if the user input is QUIT else returns true.
bool interface(std::vector<std::string> input_elements,
               std::map<std::string,
               std::vector<std::vector<std::string>>>& gig_data)
{
    std::string command = input_elements.at(0);
    if (command == "ADD_ARTIST")
    {
        add_artist(gig_data, input_elements.at(1));
    }
    else if (command == "ADD_GIG")
    {
        check_user_and_add(gig_data, input_elements);
    }
    else if (command == "CANCEL")
    {
        check_and_cancel(gig_data, input_elements);
    }
    else if (command == "QUIT")
    {
        return false;
    }
    else
    {
        print(gig_data, input_elements, command);
    }
    return true;
}

int main()
{

    std::map<std::string, std::vector<std::vector<std::string>>> gig_data;
    std::string file;
    std::cout << "Give a name for input file: ";
    getline(std::cin, file);
    if (not save_data(file, gig_data))
    {
        return EXIT_FAILURE;
    }
    while(true)
    {
        bool new_round = true;
        std::string line;
        std::cout << "gigs> ";
        getline(std::cin, line);
        std::vector<std::string> input_elems = split(line, ' ');
        if (not check_user_input(input_elems))
        {
            std::cout << "Error: Invalid input." << std::endl;
        }
        else
        {
           new_round = interface(input_elems, gig_data);
        }
        if(!new_round)
        {
            break;
        }
    }
    return EXIT_SUCCESS;
}
