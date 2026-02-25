import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

# SendGrid SMTP Configuration
SMTP_SERVER = "smtp.sendgrid.net"
SMTP_PORT = 587
SENDGRID_USERNAME = "apikey"  # Use "apikey" literally
SENDGRID_API_KEY = "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"
RECEIVER_EMAIL = "trailabwildfire@gmail.com"
SENDER_EMAIL = "ian.keefe@robotics.utias.utoronto.ca"

# Email Content
subject = "TESTING"
body = "Your PC has just booted up."

msg = MIMEMultipart()
msg["From"] = SENDER_EMAIL
msg["To"] = RECEIVER_EMAIL
msg["Subject"] = subject
msg.attach(MIMEText(body, "plain"))

# Send Email
while True:
    try:
        with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as server:
            server.starttls()
            server.login(SENDGRID_USERNAME, SENDGRID_API_KEY)
            server.sendmail(SENDER_EMAIL, RECEIVER_EMAIL, msg.as_string())
            print("Email sent successfully.")
            break
    except Exception as e:
        print(f"Failed to send email: {e}")
