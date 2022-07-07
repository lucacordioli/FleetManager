"""!@package telegram_bot_package
@file telegram_bot_node.py
@brief Main file of the telegram bot.
"""

from telegram.ext import (
    Updater,
    CommandHandler,
    MessageHandler,
    Filters,
)
from .botCallbacks.deliver import *
from .data.globaldata import *
from .botCallbacks.general import *
from .rosDialog.publisher import *


def main() -> None:
    """!@brief Main function of the telegram bot.
    Initialize and start the bot.
    Define states in the conversation with bot.
    """
    updater = Updater("5335220744:AAGHzTNu84H374Ze2TsgoF8EMySroUM6bOU")

    loadOffices()
    initRos()

    # Add conversation handler with the states GENDER, PHOTO, LOCATION and BIO
    conv_handler = ConversationHandler(
        entry_points=[CommandHandler('deliver', deliver)],
        states={
            PICKUP: [MessageHandler(Filters.text & ~Filters.command, pickup)],
            DESTINATION: [MessageHandler(Filters.text & ~Filters.command, destination)],
            SLOTS: [MessageHandler(Filters.regex('^[0-9]'), slots)],
            PRIORITY: [MessageHandler(Filters.regex('^(High|Medium|Low)$'), priority)],
        },
        fallbacks=[CommandHandler('cancel', cancel)],
    )
    updater.dispatcher.add_handler(conv_handler)
    updater.dispatcher.add_handler(CommandHandler('start', start))
    updater.dispatcher.add_handler(CommandHandler('help', help))
    updater.dispatcher.add_handler(MessageHandler(Filters.text, unknown))
    updater.dispatcher.add_handler(MessageHandler(
        # Filters out unknown commands
        Filters.command, unknown))
    # Filters out unknown messages.
    updater.dispatcher.add_handler(MessageHandler(Filters.text, unknown_text))

    updater.start_polling()
    updater.idle()

    destroyRos()


if __name__ == '__main__':
    main()
