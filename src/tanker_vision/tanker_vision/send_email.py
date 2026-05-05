import smtplib
import logging
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

logger = logging.getLogger(__name__)


class EmailError(Exception):
    pass


def send_email(
    subject: str,
    body: str,
    *,
    username: str,
    password: str,
    sender: str,
    recipient: str,
    smtp_host: str = 'smtp.gmail.com',
    smtp_port: int = 587,
) -> None:
    """
    Send a plain-text email via Gmail SMTP.

    Credentials come from config/tankervision.yaml (notifications section)
    and are passed in by the caller — nothing is hardcoded here.

    Raises:
        EmailError: on missing credentials or SMTP failure.
    """
    if not all([username, password, sender, recipient]):
        raise EmailError('Email credentials or recipient not fully configured')

    msg = MIMEMultipart()
    msg['From']    = sender
    msg['To']      = recipient
    msg['Subject'] = subject
    msg.attach(MIMEText(body, 'plain'))

    try:
        with smtplib.SMTP(smtp_host, smtp_port) as server:
            server.ehlo()
            server.starttls()
            server.ehlo()
            server.login(username, password)
            server.send_message(msg)
    except Exception as e:
        raise EmailError('Failed to send email') from e
