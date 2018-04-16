from rostopics_to_timeseries.TopicMsgFilter import TopicMsgFilter

class RosTopicFilteringConfig(object):
    def __init__(self):
        pass
        self.filters = []

    def add_filter(self, topic_name, msg_type, filter):
        if not issubclass(type(filter), TopicMsgFilter):
            raise Exception("Message filter class of no.%s info in topic_filtering_config is not a subclass of TopicMsgFilter.")
        self.filters.append(
            (
                topic_name,
                msg_type,
                filter,
            )
        )

    def iter_filters(self):
        for i in self.filters:     
            yield i[0], i[1], i[2]
    

    def get_timeseries_header(self):
        names = []
        for tn, mt, fi in self.iter_filters():
            names.extend(fi.vector_meaning())
        return names

    def get_timeseries_size(self):
        size = 0
        for tn, mt, fi in self.iter_filters():
            size += fi.vector_size()
        return size

    def get_filter_amount(self):
        return len(self.filters)
