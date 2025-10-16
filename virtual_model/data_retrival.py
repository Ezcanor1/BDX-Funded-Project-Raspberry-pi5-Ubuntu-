import requests
import json
from bs4 import BeautifulSoup
import re

# --- Installation ---
# Before running, make sure you have the required libraries installed:
# pip install requests beautifulsoup4

def fetch_website_content(url):
    """
    Fetches the HTML content of a given URL.

    Args:
        url (str): The URL of the website to scrape.

    Returns:
        str: The HTML content of the page, or None if the request fails.
    """
    try:
        print(f"Fetching content from {url}...")
        response = requests.get(url, headers={'User-Agent': 'Mozilla/5.0'})
        response.raise_for_status()  # Raise an exception for bad status codes (4xx or 5xx)
        print("Successfully fetched content.")
        return response.text
    except requests.exceptions.RequestException as e:
        print(f"Error fetching URL {url}: {e}")
        return None

def extract_meaningful_text(html_content):
    """
    Parses HTML and extracts clean, readable text.

    Args:
        html_content (str): The raw HTML of the page.

    Returns:
        str: A string containing the cleaned text from the page.
    """
    if not html_content:
        return ""
    
    print("Parsing HTML and extracting text...")
    soup = BeautifulSoup(html_content, 'html.parser')

    # Remove script and style elements
    for script_or_style in soup(['script', 'style']):
        script_or_style.decompose()

    # Get text and clean it up
    text = soup.get_text()
    lines = (line.strip() for line in text.splitlines())
    chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
    cleaned_text = '\n'.join(chunk for chunk in chunks if chunk)
    print("Text extraction complete.")
    return cleaned_text

def process_with_ai_agent(text):
    """
    Simulates an AI agent that structures the extracted text.

    This function looks for common keywords to identify sections like
    'About Us', 'Admissions', 'Courses', etc., and extracts the
    text that follows them.

    For a more powerful solution, you could replace this function with a
    call to a real AI model (like Google's Gemini API). You would send the
    'text' to the model and ask it to return a structured JSON.

    Args:
        text (str): The cleaned text from the website.

    Returns:
        dict: A dictionary with structured data.
    """
    print("AI Agent is processing the text...")
    data = {
        'title': '',
        'about': '',
        'admissions': '',
        'courses': [],
        'contact': ''
    }
    
    # Simple logic to find the page title (often the first non-empty line)
    lines = text.split('\n')
    if lines:
        data['title'] = lines[0]

    # Use regular expressions to find sections. This is a basic simulation.
    # A real AI would understand context much better.
    about_match = re.search(r'(About Us|About the College|Welcome to)([\s\S]*?)(Admissions|Courses|Contact Us|©)', text, re.IGNORECASE)
    if about_match:
        data['about'] = about_match.group(2).strip()

    admissions_match = re.search(r'(Admissions|Admission Process)([\s\S]*?)(Courses|Departments|Contact Us|©)', text, re.IGNORECASE)
    if admissions_match:
        data['admissions'] = admissions_match.group(2).strip()

    contact_match = re.search(r'(Contact Us|Get in Touch)([\s\S]*?)(©|Location|Map)', text, re.IGNORECASE)
    if contact_match:
        data['contact'] = contact_match.group(2).strip()
        
    # Example for extracting a list of courses
    courses_match = re.search(r'(Courses Offered|Our Programs|Departments)([\s\S]*?)(Contact Us|About Us|Admissions|©)', text, re.IGNORECASE)
    if courses_match:
        # This is a very simple way to find potential courses, splitting by newline
        potential_courses = courses_match.group(2).strip().split('\n')
        data['courses'] = [course.strip() for course in potential_courses if len(course.strip()) > 3 and not course.strip().startswith(('Read More', 'Learn More'))]

    print("AI Agent processing finished.")
    return data


def save_to_json(data, filename="college_data.json"):
    """
    Saves a dictionary to a JSON file.

    Args:
        data (dict): The data to save.
        filename (str): The name of the output file.
    """
    print(f"Saving data to {filename}...")
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=4, ensure_ascii=False)
    print("Data saved successfully.")


def main():
    """
    Main function to run the data extraction process.
    """
    # !!! IMPORTANT !!!
    # Replace this URL with your college's website URL.
    college_url = "https://www.pdacek.ac.in/" # <--- CHANGE THIS

    # 1. Fetch the HTML content from the website
    html = fetch_website_content(college_url)

    if html:
        # 2. Extract clean text from the HTML
        page_text = extract_meaningful_text(html)

        # 3. Process the text with our "AI Agent" to structure it
        structured_data = process_with_ai_agent(page_text)

        # 4. Save the structured data to a JSON file
        save_to_json(structured_data)

        # Optional: Print the final JSON to the console
        print("\n--- Extracted Data ---")
        print(json.dumps(structured_data, indent=2))
        print("\n----------------------")


if __name__ == "__main__":
    main()
