import os
import logging
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from typing import Optional

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class EmailError(Exception):
    """Custom exception for email sending errors."""
    pass


def send_email(
    subject: str,
    body: str
) -> None:
    """
    Send an email using SMTP (configured for SendGrid by default).

    Parameters:
        subject: Subject line of the email.
        body: Plain-text body of the email.EY.
    Raises:
        EmailError: If sending fails or configuration is missing.
    """
    # Load configuration from environment if not provided
    SMTP_SERVER = "smtp.sendgrid.net"
    SMTP_PORT = 587
    SENDGRID_USERNAME = "apikey"  # Use "apikey" literally
    SENDGRID_API_KEY = "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"
    RECEIVER_EMAIL = "trailabwildfire@gmail.com"
    SENDER_EMAIL = "ian.keefe@robotics.utias.utoronto.ca"
    # Build message
    msg = MIMEMultipart()
    msg['From'] = SENDER_EMAIL
    msg['To'] = RECEIVER_EMAIL
    msg['Subject'] = subject
    msg.attach(MIMEText(body, 'plain'))

    try:
        with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as server:
            server.ehlo()
            server.starttls()
            server.ehlo()
            server.login(SENDGRID_USERNAME, SENDGRID_API_KEY)
            server.send_message(msg)
    except Exception as e:
        raise EmailError('Failed to send email') from e


if __name__ == '__main__':
    # Example usage
    try:
        send_email(
            subject='Test Email',
            body='Your PC has just booted up.',
        )
    except EmailError as err:
        logger.error('Error sending email: %s', err)
