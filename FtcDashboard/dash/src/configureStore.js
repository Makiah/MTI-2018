import { applyMiddleware, createStore } from 'redux';
import { createLogger } from 'redux-logger';
import thunk from 'redux-thunk';
import socketMiddleware from './socketMiddleware';
import reducer from './reducers';
import { RECEIVE_PING_TIME } from './actions/socket';

const logger = createLogger({
  predicate: (getState, action) => (action.type !== RECEIVE_PING_TIME)
});

const configureStore = () => (
  createStore(
    reducer,
    applyMiddleware(thunk, socketMiddleware, logger)
  )
);

export default configureStore;